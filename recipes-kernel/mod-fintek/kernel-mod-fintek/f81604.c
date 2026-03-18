/*
 * CAN driver for "Fintek F81603/604" USB-to-1/2CAN converter
 *
 * Copyright (C) 2018 Ji-Ze Hong (Peter Hong) (hpeter+linux_kernel@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * This driver is inspired by linux/drivers/net/can/usb/usb_8dev.c
 */

#define DIABLE_CANLED 1 // for RHEL9 kernel 5.14

#include <linux/netfilter.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/version.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#if !DIABLE_CANLED && LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
#include <linux/can/led.h>
#endif
#include <linux/can/dev.h>
#include <linux/can/platform/sja1000.h>

#include "sja1000.h"

#ifndef CAN_CTRLMODE_BERR_REPORTING
#define CAN_CTRLMODE_BERR_REPORTING 0x10 /* Bus-error reporting */
#define CAN_CTRLMODE_ONE_SHOT 0x08 /* One-Shot mode */
#endif

#define DRV_VER "v1.24-20251117"
#define F81604_DUMP_REG 0
#define F81604_POLL_MODE 0
#define F81604_CLEAR_ERROR_NEW 0
#define F81604_WRITE_READ_TEST 0
#define F81604_HWDISABLE 1
#define F81604_MONITOR_WRITE_BULK 0
#define F81604_INT_WAKE_TX 0

/* vendor and product id */
#define F81604_VENDOR_ID 0x2c42
#define F81604_PRODUCT_ID 0x1709

#define F81604_USB_MAX_RETRY 10
#define F81604_USB_TIMEOUT 2000
#define F81604_SET_GET_REGISTER 0xA0
#define F81604_PORT_OFFSET 0x1000

#define F81604_BULK_SIZE 64
#define F81604_INT_SIZE 16
#define F81604_READ_URB_SIZE 3
#define F81604_READ_REG_URB_SIZE 2 //16
#define F81604_DATA_SIZE 14

#define F81604_READ_REG_OFFSET 2

#define F81604_MAX_RX_REG_URBS 2
#define F81604_MAX_RX_URBS 4

#define F81604_CMD_OFFSET 0x00
#define F81604_CMD_DATA 0x00
#define F81604_CMD_READ_REG 0x01
#define F81604_CMD_WRITE_REG 0x02

#define F81604_DLC_OFFSET 0x01
#define F81604_LEN_MASK 0x0f
#define F81604_EFF_BIT BIT(7)
#define F81604_RTR_BIT BIT(6)

#define F81604_ID1_OFFSET 0x02
#define F81604_ID2_OFFSET 0x03
#define F81604_ID3_OFFSET 0x04
#define F81604_ID4_OFFSET 0x05

#define F81604_SFF_DATA_OFFSET 0x04
#define F81604_EFF_DATA_OFFSET 0x06

/* device CAN clock */
#define F81604_V1_CLOCK (16000000 / 2)
#define F81604_V2_CLOCK (24000000 / 2)
#define F81604_MAX_DEV 2

#define F81604_TERMINATOR_REG 0x105
#define F81604_CAN0_TERM BIT(2)
#define F81604_CAN1_TERM BIT(3)

#define F81604_HW_VERSION 0x10f

struct f81604_reg_ctrl {
	unsigned int reg;
	unsigned char data;
};

#define FINTEK_IOCTL_WR_REG (SIOCDEVPRIVATE + 0)
#define FINTEK_IOCTL_RD_REG (SIOCDEVPRIVATE + 1)

/* table of devices that work with this driver */
static const struct usb_device_id f81604_table[] = {
	{ USB_DEVICE(F81604_VENDOR_ID, F81604_PRODUCT_ID) },
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, f81604_table);

struct f81604_priv {
	struct net_device *netdev[F81604_MAX_DEV];
	struct mutex mutex;
};

struct f81604_port_priv {
	struct can_priv can;
	struct net_device *netdev;
	struct sk_buff *echo_skb;
#if F81604_DUMP_REG
	u8 alc, ecc, ali;
	struct delayed_work dump_reg_delayed_work;
#endif
#if F81604_POLL_MODE
	struct delayed_work poll_delayed_work;
#endif
#if F81604_MONITOR_WRITE_BULK
	struct delayed_work write_bulk_monitor_delayed_work;
	unsigned int current_sr_idx;
#endif
	spinlock_t lock;
	bool need_clear_alc;
	bool need_clear_ecc;

	struct work_struct handle_int_work;
	struct work_struct handle_clear_reg_work;
	struct work_struct handle_clear_overrun_work;

	struct usb_device *dev;
	struct usb_interface *intf;

	struct urb *int_urb;
	u8 *int_read_buffer;

	struct urb *read_sr_urb[F81604_MAX_RX_REG_URBS];
	u8 *bulk_read_sr_buffer[F81604_MAX_RX_REG_URBS];
	unsigned long read_sr_flag;

	struct urb *read_urb[F81604_MAX_RX_URBS];
	u8 *bulk_read_buffer[F81604_MAX_RX_URBS];

	struct urb *write_urb;
	u8 *bulk_write_buffer;
	bool is_txing;

#if F81604_WRITE_READ_TEST
	struct urb *write_reg_urb;
	u8 *bulk_write_reg_buffer;
#endif

	int tx_size;
	u8 ocr;
	u8 cdr;

	int ver;
};

#ifndef get_can_dlc
#define get_can_dlc can_cc_dlc2len
#endif

#if F81604_MONITOR_WRITE_BULK
static unsigned int write_bulk_timeout = 5; //ms
module_param(write_bulk_timeout, uint, S_IRUGO);
MODULE_PARM_DESC(write_bulk_timeout, "TX write bulk timeout check");
#endif

#if F81604_INT_WAKE_TX
static unsigned int int_wake_tx = 0;
module_param(int_wake_tx, uint, S_IRUGO);
MODULE_PARM_DESC(int_wake_tx, "int_wake_tx");
#endif

static unsigned int delay_reg_sr_us = 0;
module_param(delay_reg_sr_us, uint, S_IRUGO);
MODULE_PARM_DESC(delay_reg_sr_us, "delay_reg_sr_us");

static unsigned int hwdisable_detect = 1;
module_param(hwdisable_detect, uint, S_IRUGO);
MODULE_PARM_DESC(hwdisable_detect, "hwdisable_detect");

static unsigned int force_sjw_max = 1;
module_param(force_sjw_max, uint, S_IRUGO);
MODULE_PARM_DESC(force_sjw_max, "force_sjw_max");

static unsigned int force_sp_max = 1;
module_param(force_sp_max, uint, S_IRUGO);
MODULE_PARM_DESC(force_sp_max, "force_sp_max");

static unsigned int more_err_report = 0;
module_param(more_err_report, uint, S_IRUGO);
MODULE_PARM_DESC(more_err_report, "more_err_report");

static void f81604_unregister_urbs(struct net_device *netdev);
static int f81604_register_urbs(struct net_device *netdev);

static int f81604_set_register(struct usb_device *dev, u16 reg, u8 data)
{
	size_t count = F81604_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	*tmp = data;

	while (count--) {
		status = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
					 F81604_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_OUT, 0, reg,
					 tmp, sizeof(u8), F81604_USB_TIMEOUT);
		if (status > 0) {
			status = 0;
			break;
		} else if (status == 0) {
			status = -EIO;
		}
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s: reg: %x data: %x failed: %d\n",
			__func__, reg, data, status);
	}

	kfree(tmp);
	return status;
}

static int f81604_get_register(struct usb_device *dev, u16 reg, u8 *data)
{
	size_t count = F81604_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	while (count--) {
		status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
					 F81604_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_IN, 0, reg,
					 tmp, sizeof(u8), F81604_USB_TIMEOUT);
		if (status > 0) {
			status = 0;
			break;
		} else if (status == 0) {
			status = -EIO;
		}
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s: reg: %x failed: %d\n", __func__, reg,
			status);
		goto end;
	}

	*data = *tmp;

end:
	kfree(tmp);
	return status;
}

static int f81604_mask_set_register(struct usb_device *dev, u16 reg, u8 mask,
				    u8 data)
{
	int status;
	u8 tmp;

	status = f81604_get_register(dev, reg, &tmp);
	if (status)
		return status;

	tmp &= ~mask;
	tmp |= (mask & data);

	return f81604_set_register(dev, reg, tmp);
}

static int f81604_set_sja1000_register(struct usb_device *dev, u8 port,
				       u16 reg, u8 data)
{
	return f81604_set_register(
		dev, reg + F81604_PORT_OFFSET * port + F81604_PORT_OFFSET,
		data);
}

static int f81604_get_sja1000_register(struct usb_device *dev, u8 port,
				       u16 reg, u8 *data)
{
	return f81604_get_register(
		dev, reg + F81604_PORT_OFFSET * port + F81604_PORT_OFFSET,
		data);
}

static ssize_t terminator_control_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct net_device *netdev;
	struct f81604_port_priv *port_priv;
	struct f81604_priv *priv;
	int status;
	u8 mask, data = 0;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);

	if (netdev->dev_id == 0) {
		mask = F81604_CAN0_TERM;

		if (buf[0] == '1')
			data = F81604_CAN0_TERM;
	} else {
		mask = F81604_CAN1_TERM;

		if (buf[0] == '1')
			data = F81604_CAN1_TERM;
	}

	mutex_lock(&priv->mutex);
	status = f81604_mask_set_register(port_priv->dev,
					  F81604_TERMINATOR_REG, mask, data);
	mutex_unlock(&priv->mutex);

	if (status)
		return -EINVAL;

	return count;
}

static ssize_t terminator_control_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct net_device *netdev;
	struct f81604_port_priv *port_priv;
	struct f81604_priv *priv;
	int status;
	u8 mask, data;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);

	if (netdev->dev_id == 0)
		mask = F81604_CAN0_TERM;
	else
		mask = F81604_CAN1_TERM;

	mutex_lock(&priv->mutex);
	status = f81604_get_register(port_priv->dev, F81604_TERMINATOR_REG,
				     &data);
	mutex_unlock(&priv->mutex);

	if (status)
		return -EINVAL;

	return sprintf(buf, "%d\n", !!(data & mask));
}

static DEVICE_ATTR_RW(terminator_control);

static ssize_t force_read_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct net_device *netdev;
	struct f81604_port_priv *port_priv;
	struct f81604_priv *priv;
	int i, status, data[2];
	u8 *p;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);

	for (i = 0; i < 2; ++i) {
		p = strsep((char **)&buf, " ");
		if (!p)
			break;

		status = kstrtoint(p, 0, &data[i]);
		if (status)
			break;
	}

	if (i == 0)
		return status;

	if (i == 1) {
		f81604_get_sja1000_register(port_priv->dev, netdev->dev_id,
					    data[0], (u8 *)&data[1]);
		pr_info("netdev->dev_id: %d, read reg: %x, data: %x\n",
			netdev->dev_id, data[0], data[1]);
	} else {
		pr_info("netdev->dev_id: %d, write reg: %x, data: %x\n",
			netdev->dev_id, data[0], data[1]);
		f81604_set_sja1000_register(port_priv->dev, netdev->dev_id,
					    data[0], data[1]);
	}

	return count;
}

static ssize_t force_read_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct net_device *netdev;
	struct f81604_port_priv *port_priv;
	struct f81604_priv *priv;
	u8 data;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);

	f81604_get_sja1000_register(port_priv->dev, netdev->dev_id, 0x0a,
				    &data);

	return sprintf(buf, "ok\n");
}

static DEVICE_ATTR_RW(force_read);

static ssize_t force_tx_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct net_device *netdev;
	struct f81604_port_priv *port_priv;
	struct f81604_priv *priv;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);

#if 0
	f81604_get_sja1000_register(port_priv->dev, netdev->dev_id, 0x0a,
					&data);
#else
	netdev_info(netdev, "%s: is stopped: %d\n", __func__,
		    netif_queue_stopped(netdev));
	netif_wake_queue(netdev);
#endif
	return sprintf(buf, "ok\n");
}

static DEVICE_ATTR_RO(force_tx);

static int f81604_set_reset_mode(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	u8 tmp;
	int status;
	int i;

	/* disable interrupts */
	status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_IER, IRQ_OFF);
	if (status)
		return status;

	for (i = 0; i < 100; i++) {
		status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, &tmp);
		if (status)
			return status;

		/* check reset bit */
		if (tmp & MOD_RM) {
			priv->can.state = CAN_STATE_STOPPED;
			return 0;
		}

		/* reset chip */
		status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, MOD_RM);
		if (status)
			return status;
	}

	netdev_err(netdev, "setting SJA1000 into reset mode failed!\n");
	return -EINVAL;
}

static int f81604_set_normal_mode(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int status;
	u8 tmp;
	u8 mod_reg_val = 0x00;
	u8 ier = 0;
	int i;

	for (i = 0; i < 100; i++) {
		status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, &tmp);
		if (status)
			return status;

		/* check reset bit */
		if ((tmp & MOD_RM) == 0) {
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
			/* enable interrupts */
			if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) {
				ier = IRQ_ALL & ~(IRQ_TI | IRQ_RI);
#if F81604_INT_WAKE_TX
				if (int_wake_tx)
					ier |= IRQ_TI;
#endif
				if (!more_err_report)
					ier &= ~(IRQ_ALI | IRQ_BEI);

				status = f81604_set_sja1000_register(
					priv->dev, netdev->dev_id, SJA1000_IER,
					ier);
				if (status)
					return status;
			} else {
				ier = IRQ_ALL & ~(IRQ_TI | IRQ_RI | IRQ_BEI);
#if F81604_INT_WAKE_TX
				if (int_wake_tx)
					ier |= IRQ_TI;
#endif
				status = f81604_set_sja1000_register(
					priv->dev, netdev->dev_id, SJA1000_IER,
					ier);
				if (status)
					return status;
			}

			return 0;
		}

		/* set chip to normal mode */
		if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
			mod_reg_val |= MOD_LOM;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
		if (priv->can.ctrlmode & CAN_CTRLMODE_PRESUME_ACK)
			mod_reg_val |= MOD_STM;
#endif
		status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, mod_reg_val);
		if (status)
			return status;

		udelay(10);
	}

	netdev_err(netdev, "setting SJA1000 into normal mode failed!\n");
	return -EINVAL;
}

/*
 * initialize SJA1000 chip:
 *   - reset chip
 *   - set output mode
 *   - set baudrate
 *   - enable interrupts
 *   - start operating mode
 */
static int f81604_chipset_init(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	struct can_bittiming *bt = &priv->can.bittiming;
	int status;
	int i;

	if (bt->bitrate >= 500000)
		priv->cdr |= BIT(3);
	else
		priv->cdr &= ~BIT(3);

	/* set clock divider and output control register */
	status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_CDR,
					     priv->cdr | CDR_PELICAN);
	if (status)
		return status;

	/* set acceptance filter (accept all) */
	for (i = 0; i < 4; ++i) {
		status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_ACCC0 + i, 0);
		if (status)
			return status;
	}

	for (i = 0; i < 4; ++i) {
		status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_ACCM0 + i, 0xFF);
		if (status)
			return status;
	}

	return f81604_set_sja1000_register(priv->dev, netdev->dev_id,
					   SJA1000_OCR,
					   priv->ocr | OCR_MODE_NORMAL);
}

static int f81604_start(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int status;
	u8 tmp;
	u8 mode;

	f81604_unregister_urbs(netdev);

	/* Set TR/AT mode */
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		mode = 0x1b;
	else
		mode = 0x0b; //0x0b;

	status = f81604_set_sja1000_register(priv->dev, netdev->dev_id, 0x80,
					     mode);
	if (status)
		return status;

	/* set reset mode */
	status = f81604_set_reset_mode(netdev);
	if (status)
		return status;

	/* Initialize chip if uninitialized at this stage */
	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_CDR, &tmp);
	if (status)
		return status;

	status = f81604_chipset_init(netdev);
	if (status)
		return status;

	/* Clear error counters and error code capture */
	status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_TXERR, 0);
	if (status)
		return status;

	status = f81604_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_RXERR, 0);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_ALC - 1, &tmp);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_ECC, &tmp);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_ALC, &tmp);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_IR, &tmp);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_SR, &tmp);
	if (status)
		return status;

	f81604_register_urbs(netdev);

	/* leave reset mode */
	return f81604_set_normal_mode(netdev);
}

static int f81604_set_bittiming(struct net_device *dev)
{
	struct f81604_port_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u8 btr0, btr1, force_sjw;
	int status = 0;

	btr0 = ((bt->brp - 1) & 0x3f) | (((bt->sjw - 1) & 0x3) << 6);

	if ((bt->bitrate == 1000000) && force_sp_max) {
		btr1 = 0x09;
		btr0 &= ~(BIT(7) | BIT(6));
	} else {
		btr1 = ((bt->prop_seg + bt->phase_seg1 - 1) & 0xf) |
		       (((bt->phase_seg2 - 1) & 0x7) << 4);

		if (force_sjw_max) {
			force_sjw = min_t(int, bt->phase_seg2 - 1, 3);

			btr0 &= ~(BIT(7) | BIT(6));
			btr0 |= force_sjw << 6;
		}
	}

	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		btr1 |= 0x80;

	netdev_info(dev, "BTR0=0x%02x BTR1=0x%02x\n", btr0, btr1);

	status |= f81604_set_sja1000_register(priv->dev, dev->dev_id,
					      SJA1000_BTR0, btr0);
	status |= f81604_set_sja1000_register(priv->dev, dev->dev_id,
					      SJA1000_BTR1, btr1);

	if (status) {
		netdev_info(
			dev,
			"setting id:%d BTR0=0x%02x BTR1=0x%02x failed: %d\n",
			dev->dev_id, btr0, btr1, status);
	}

	return status;
}

static int f81604_set_mode(struct net_device *netdev, enum can_mode mode)
{
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		err = f81604_start(netdev);
		if (!err && netif_queue_stopped(netdev))
			netif_wake_queue(netdev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return err;
}

static int f81604_register_read_sr_urbs(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int status;
	int i;

	//netdev_info(netdev, "%s: in\n", __func__);

	for (i = 0; i < F81604_MAX_RX_REG_URBS; ++i) {
		if (!test_and_set_bit(i, &priv->read_sr_flag))
			break;
	}

	if (i >= F81604_MAX_RX_REG_URBS) {
		netdev_info(netdev, "%s: line: %d\n", __func__, __LINE__);
		return -EAGAIN;
	}

	priv->bulk_read_sr_buffer[i][0] = F81604_CMD_READ_REG;
	priv->bulk_read_sr_buffer[i][1] = F81604_READ_REG_OFFSET;
	priv->bulk_read_sr_buffer[i][2] = F81604_READ_REG_URB_SIZE - 1;

	//netdev_info(netdev, "%s: submit: %d\n", __func__, __LINE__);

	if (delay_reg_sr_us)
		udelay(delay_reg_sr_us);

	status = usb_submit_urb(priv->read_sr_urb[i], GFP_ATOMIC);
	if (status) {
		netdev_info(netdev, "%s: submit failed: %d %d\n", __func__,
			    __LINE__, status);
		clear_bit(i, &priv->read_sr_flag);

		return status;
	}

#if F81604_MONITOR_WRITE_BULK
	if (write_bulk_timeout) {
		priv->current_sr_idx = i;

		cancel_delayed_work(&priv->write_bulk_monitor_delayed_work);
		schedule_delayed_work(&priv->write_bulk_monitor_delayed_work,
				      msecs_to_jiffies(write_bulk_timeout));
	}
#endif

	return 0;
}

static void f81604_read_sr_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int i;

	//netdev_info(netdev, "%s: URB (%d)\n", __func__, urb->status);

	for (i = 0; i < F81604_MAX_RX_REG_URBS; ++i) {
		if (priv->read_sr_urb[i] == urb) {
			clear_bit(i, &priv->read_sr_flag);
			//netdev_info(netdev, "%s: sr release: %d\n", __func__, i);
			return;
		}
	}
}

static void f81604_process_sr_packet(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct f81604_port_priv *priv = netdev_priv(netdev);
	u8 *data = urb->transfer_buffer;
	unsigned long flags;
	u8 mask, sts;

	if (urb->actual_length != F81604_READ_REG_URB_SIZE) {
		netdev_warn(netdev, "read ir size: %d failed\n",
			    urb->actual_length);
		goto resubmit;
	}

#if F81604_MONITOR_WRITE_BULK
	if (write_bulk_timeout)
		cancel_delayed_work(&priv->write_bulk_monitor_delayed_work);
#endif

#if 0
	if (netdev->dev_id == 0){
		static u8 old_sr;

		if (old_sr != data[1]) {
			old_sr = data[1];
			netdev_info(netdev, "%s: 111 sr: %02x\n", __func__, data[1]);
		}
	}
#endif

	//busoff
	if (data[1] & BIT(7)) {
#if 0
		spin_lock_irqsave(&priv->lock, flags);
		
		if (priv->can.state != CAN_STATE_BUS_OFF) {
			priv->can.state = CAN_STATE_BUS_OFF;
			can_bus_off(netdev);
		}
		
		spin_unlock_irqrestore(&priv->lock, flags);
#endif
		goto resubmit;
	}

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->is_txing) {
		spin_unlock_irqrestore(&priv->lock, flags);
		goto resubmit;
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->can.state == CAN_STATE_STOPPED ||
	    priv->can.state == CAN_STATE_BUS_OFF)
		return;

	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) {
		mask = SR_TS | SR_TBS;
		sts = SR_TBS;
	} else {
		mask = SR_TS | SR_TBS | SR_TCS;
		sts = SR_TBS | SR_TCS;
	}

	if ((data[1] & mask) == sts) {
		//netdev_info(netdev, "%s: 222 sr: %02x netif_wake_queue\n", __func__, data[1]);
		netif_wake_queue(netdev);
		return;
	} else {
		//netdev_info(netdev, "%s: 111 sr: %02x\n", __func__, data[1]);
	}

#if 0
	if (data[1] & SR_BS)
		netdev_err(netdev, "Bus status %x %x\n", data[0], data[1]);
	else if (data[1] & SR_ES)
		netdev_err(netdev, "Error status %x %x\n", data[0], data[1]);
	else if (data[1] & SR_DOS)
		netdev_err(netdev, "Data overrun %x %x\n", data[0], data[1]);
#endif

	//print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, data,
	//		urb->actual_length, true);
	//pr_info("%s: not wake\n", __func__);

resubmit:
	f81604_register_read_sr_urbs(netdev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
struct sk_buff *alloc_can_skb(struct net_device *dev, struct can_frame **cf)
{
	struct sk_buff *skb;

	skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
	if (unlikely(!skb))
		return NULL;

	skb->protocol = htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	*cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));
	memset(*cf, 0, sizeof(struct can_frame));

	return skb;
}
#endif

static void f81604_process_rx_packet(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &netdev->stats;
	u8 *data = urb->transfer_buffer;
	u8 *ptr;
	int i;
	int count;

	if (urb->actual_length % 14) {
		netdev_warn(netdev, "actual_length %% 14 != 0 (%d)\n",
			    urb->actual_length);

		for (i = 0; i < urb->actual_length; ++i)
			netdev_warn(netdev, "data: %d %x\r\n", i, data[i]);

	} else if (!urb->actual_length) {
		netdev_warn(netdev, "actual_length = 0 (%d)\n",
			    urb->actual_length);
	}

	count = urb->actual_length / F81604_DATA_SIZE;

	for (i = 0; i < count; ++i) {
		ptr = &data[i * F81604_DATA_SIZE];

		if (ptr[F81604_CMD_OFFSET] != F81604_CMD_DATA)
			continue;

		skb = alloc_can_skb(netdev, &cf);
		if (!skb)
			continue;

		cf->can_dlc = ptr[F81604_DLC_OFFSET] & 0xF;

		if (ptr[F81604_DLC_OFFSET] & F81604_EFF_BIT) {
			cf->can_id = (ptr[F81604_ID1_OFFSET] << 21) |
				     (ptr[F81604_ID2_OFFSET] << 13) |
				     (ptr[F81604_ID3_OFFSET] << 5) |
				     (ptr[F81604_ID4_OFFSET] >> 3);
			cf->can_id |= CAN_EFF_FLAG;
		} else {
			cf->can_id = (ptr[F81604_ID1_OFFSET] << 3) |
				     (ptr[F81604_ID2_OFFSET] >> 5);
		}

		if (ptr[F81604_DLC_OFFSET] & F81604_RTR_BIT) {
			cf->can_id |= CAN_RTR_FLAG;
		} else if (ptr[F81604_DLC_OFFSET] & F81604_EFF_BIT) {
			memcpy(cf->data, &ptr[F81604_EFF_DATA_OFFSET],
			       cf->can_dlc);
		} else {
			memcpy(cf->data, &ptr[F81604_SFF_DATA_OFFSET],
			       cf->can_dlc);
		}

		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
#if !DIABLE_CANLED && LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
		can_led_event(netdev, CAN_LED_EVENT_RX);
#endif
	}

	//print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, data,
	//		urb->actual_length, true);
}

/* Callback for reading data from device
 *
 * Check urb status, call read function and resubmit urb read operation.
 */
static void f81604_read_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	u8 *data = urb->transfer_buffer;
	int status, i;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		//netdev_info(netdev, "%s: URB ok (%d)\n", __func__, urb->status);
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		netdev_dbg(netdev, "%s: URB aborted (%d)\n", __func__,
			   urb->status);
		return;

	default:
		netdev_dbg(netdev, "%s: URB aborted (%d)\n", __func__,
			   urb->status);
		goto resubmit_urb;
	}

	switch (data[F81604_CMD_OFFSET]) {
	case F81604_CMD_DATA:
		if ((urb->actual_length % 14) == 0) {
			f81604_process_rx_packet(urb);
		} else {
			netdev_warn(netdev, "%s: remap\n", __func__);

			for (i = 0; i < urb->actual_length; ++i)
				netdev_warn(netdev, "data: %d %x\r\n", i,
					    data[i]);

			f81604_process_sr_packet(urb);
		}
		break;
	case F81604_CMD_READ_REG:
		f81604_process_sr_packet(urb);
		break;
	default:
		netdev_err(netdev, "unknown header: %x, len: %d\n",
			   data[F81604_CMD_OFFSET], urb->actual_length);
		break;
	}

resubmit_urb:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status == -ENODEV) {
		netif_device_detach(netdev);
	} else if (status) {
		netdev_err(netdev, "failed resubmitting read bulk urb: %d\n",
			   status);
	}
}

/* Callback handler for write operations
 *
 * Free allocated buffers, check transmit status and
 * calculate statistic.
 */
static void f81604_write_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct f81604_port_priv *priv = netdev_priv(netdev);
	unsigned long flags;
	int r;

	spin_lock_irqsave(&priv->lock, flags);

	if (!netif_device_present(netdev)) {
		priv->is_txing = false;
		spin_unlock_irqrestore(&priv->lock, flags);
		return;
	}

	switch (urb->status) {
	case 0: /* success */
		//netdev_info(netdev, "%s: URB ok (%d)\n", __func__, urb->status);
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		priv->is_txing = false;
		spin_unlock_irqrestore(&priv->lock, flags);
		netdev_dbg(netdev, "%s: URB aborted (%d)\n", __func__,
			   urb->status);
		return;

	default:
		netdev_dbg(netdev, "Tx URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}

	if (priv->tx_size) {
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += priv->tx_size;
		priv->tx_size = 0;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	r = can_get_echo_skb(netdev, 0, NULL);
#else
	r = can_get_echo_skb(netdev, 0);
#endif
#else
	can_get_echo_skb(netdev, 0);
#endif

#if !DIABLE_CANLED && LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
	can_led_event(netdev, CAN_LED_EVENT_TX);
#endif

resubmit_urb:
#if F81604_INT_WAKE_TX
	if (!int_wake_tx)
#endif
		f81604_register_read_sr_urbs(netdev);

	priv->is_txing = false;
	spin_unlock_irqrestore(&priv->lock, flags);
}

#if F81604_WRITE_READ_TEST
static void f81604_write_reg_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int status;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		//netdev_info(netdev, "%s: URB ok (%d)\n", __func__, urb->status);
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		netdev_dbg(netdev, "%s: URB aborted (%d)\n", __func__,
			   urb->status);
		return;

	default:
		netdev_dbg(netdev, "%s: Tx URB aborted (%d)\n", __func__,
			   urb->status);
	}

	status = usb_submit_urb(priv->write_reg_urb, GFP_ATOMIC);
	if (status)
		pr_err("%s: submit write_reg urb failed: %d\n", __func__,
		       status);
}
#endif

#if F81604_POLL_MODE
static int f81604_int_handler(struct work_struct *work)
{
	struct f81604_port_priv *priv =
		container_of(work, struct f81604_port_priv, handle_int_work);
	struct net_device *netdev = priv->can.dev;
	struct net_device_stats *stats = &netdev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state state = priv->can.state;
	enum can_state rx_state, tx_state;
	int status;
	u8 rxerr, txerr;
	u8 ecc, alc;
	u8 isrc, sr;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_IR, &isrc);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_SR, &sr);
	if (status)
		return status;

	/* Handle TR/AT TX complete */
	if ((priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) && (isrc & IRQ_TI))
		netif_wake_queue(netdev);

	if (!(isrc & (IRQ_DOI | IRQ_EI | IRQ_BEI | IRQ_EPI | IRQ_ALI))) {
		/* no error interrupt */
		return 0;
	}

	skb = alloc_can_err_skb(netdev, &cf);
	if (skb == NULL) {
		netdev_warn(netdev, "no memory to alloc_can_err_skb\n");
		return -EINVAL;
	}

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_TXERR, &txerr);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_RXERR, &rxerr);
	if (status)
		return status;

	cf->data[6] = txerr;
	cf->data[7] = rxerr;

	//pr_info("id: %d, err sr: %x, ir: %x\n", netdev->dev_id, sr, isrc);

	if (isrc & IRQ_DOI) {
		/* data overrun interrupt */
		netdev_dbg(netdev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;

		f81604_set_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_CMR, CMD_CDO);
	}

	if (isrc & IRQ_EI) {
		if (sr & SR_BS)
			state = CAN_STATE_BUS_OFF;
		else if (sr & SR_ES)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_ACTIVE;

		/* error warning interrupt */
		netdev_dbg(netdev, "error warning interrupt %x, status: %d\n",
			   sr, state);
	}

	if (isrc & IRQ_BEI) {
		/* bus error interrupt */
		netdev_dbg(netdev, "bus error interrupt\n");

		priv->can.can_stats.bus_error++;
		stats->rx_errors++;

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		/* set error type */
		switch (ecc & ECC_MASK) {
		case ECC_BIT:
			netdev_dbg(netdev, "Bus Error: ECC_BIT %x\n", sr);
			cf->data[2] |= CAN_ERR_PROT_BIT;
			break;
		case ECC_FORM:
			netdev_dbg(netdev, "Bus Error: ECC_FORM %x\n", sr);
			cf->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case ECC_STUFF:
			netdev_dbg(netdev, "Bus Error: ECC_STUFF %x\n", sr);
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		default:
			break;
		}

		f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_ECC, &ecc);

		/* set error location */
		cf->data[3] = ecc & ECC_SEG;

		/* Error occurred during transmission? */
		if ((ecc & ECC_DIR) == 0)
			cf->data[2] |= CAN_ERR_PROT_TX;

		f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_ECC - 1, &ecc);
	}

	if (isrc & IRQ_EPI) {
		if (state == CAN_STATE_ERROR_PASSIVE)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_PASSIVE;

		/* error passive interrupt */
		netdev_dbg(netdev, "error passive interrupt: %x\n", state);
	}

	if (isrc & IRQ_ALI) {
		/* arbitration lost interrupt */
		netdev_dbg(netdev, "arbitration lost interrupt\n");

		f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_ALC, &alc);
		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = alc & 0x1f;

		f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_ALC - 1, &alc);
	}

	if (state != priv->can.state) {
		tx_state = txerr >= rxerr ? state : 0;
		rx_state = txerr <= rxerr ? state : 0;

		can_change_state(netdev, cf, tx_state, rx_state);

		if (state == CAN_STATE_BUS_OFF)
			can_bus_off(netdev);
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);

	return -EAGAIN;
}

static void f81604_int_handle_work(struct work_struct *work)
{
	// while (f81604_int_handler(work) == -EAGAIN);
	f81604_int_handler(work);
}
#endif

static void f81604_handle_clear_overrun_work(struct work_struct *work)
{
	struct f81604_port_priv *priv = container_of(
		work, struct f81604_port_priv, handle_clear_overrun_work);
	struct net_device *netdev = priv->netdev;

	f81604_set_sja1000_register(priv->dev, netdev->dev_id, SJA1000_CMR,
				    CMD_CDO);
}

static void f81604_handle_clear_reg_work(struct work_struct *work)
{
	struct f81604_port_priv *priv = container_of(
		work, struct f81604_port_priv, handle_clear_reg_work);
	struct net_device *netdev = priv->netdev;
	bool clear_ecc, clear_alc;
	unsigned long flags;
	u8 tmp;
	int read_clear_offset = 0;

	spin_lock_irqsave(&priv->lock, flags);
	clear_alc = priv->need_clear_alc;
	clear_ecc = priv->need_clear_ecc;
	priv->need_clear_alc = false;
	priv->need_clear_ecc = false;
	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->ver == 1)
		read_clear_offset = 1;

	if (clear_alc) {
		f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_ALC - read_clear_offset,
					    &tmp);
		netdev_dbg(netdev, "clear ALC, %d\n", read_clear_offset);
	}

	if (clear_ecc) {
		f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_ECC - read_clear_offset,
					    &tmp);
		netdev_dbg(netdev, "clear ECC, %d\n", read_clear_offset);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
struct sk_buff *alloc_can_err_skb(struct net_device *dev,
				  struct can_frame **cf)
{
	struct sk_buff *skb;

	skb = alloc_can_skb(dev, cf);
	if (unlikely(!skb))
		return NULL;

	(*cf)->can_id = CAN_ERR_FLAG;
	(*cf)->can_dlc = CAN_ERR_DLC;

	return skb;
}
#endif

/* Read data format: SR/IR/IER/ALC/ECC/EWLR/RXERR/TXERR/VAL */
static void f81604_read_int_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct f81604_port_priv *priv = netdev_priv(netdev);
	struct net_device_stats *stats = &netdev->stats;
	enum can_state can_state = priv->can.state;
	enum can_state rx_state, tx_state;
	struct can_frame *cf;
	struct sk_buff *skb;
	u8 *data = urb->transfer_buffer;
	u8 sr, isrc, alc, ecc, rxerr, txerr, val;
	unsigned long flags;
	int status;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		netdev_dbg(netdev, "%s: URB aborted (%d)\n", __func__,
			   urb->status);
		return;

	default:
		netdev_dbg(netdev, "%s: URB aborted (%d)\n", __func__,
			   urb->status);
		goto resubmit_urb;
	}

	sr = data[0];
	isrc = data[1];
	alc = data[3];
	ecc = data[4];
	rxerr = data[6];
	txerr = data[7];
	val = data[8];

#if 0
	if (netdev->dev_id == 0){
		static u8 old_sr;

		if (old_sr != data[0]) {
			old_sr = data[0];
			netdev_info(netdev, "%s: 111 sr: %02x\n", __func__, data[0]);
		}
	}
#endif

#if F81604_INT_WAKE_TX
	if (int_wake_tx && (isrc & IRQ_TI)) {
		u8 mask, sts;

		if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) {
			mask = SR_TS | SR_TBS;
			sts = SR_TBS;
		} else {
			mask = SR_TS | SR_TBS | SR_TCS;
			sts = SR_TBS | SR_TCS;
		}

		if ((sr & mask) == sts)
			netif_wake_queue(netdev);
	}
#endif

	if (!(isrc & (IRQ_DOI | IRQ_EI | IRQ_BEI | IRQ_EPI | IRQ_ALI))) {
		/* no error interrupt */
		//netdev_info(netdev, "%s: 222 sr: %02x, ir: %x\n", __func__, data[0], isrc);
		goto resubmit_urb;
	}

	//netdev_info(netdev, "%s: bbb sr: %x, ir: %x\n", __func__, sr, isrc);

	skb = alloc_can_err_skb(netdev, &cf);
	if (skb == NULL) {
		netdev_warn(netdev, "no memory to alloc_can_err_skb\n");
		goto resubmit_urb;
	}

	cf->data[6] = txerr;
	cf->data[7] = rxerr;

	if (isrc & IRQ_DOI) {
		/* data overrun interrupt */
		netdev_dbg(netdev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;

		schedule_work(&priv->handle_clear_overrun_work);
	}

	if (isrc & IRQ_EI) {
		if (sr & SR_BS)
			can_state = CAN_STATE_BUS_OFF;
		else if (sr & SR_ES)
			can_state = CAN_STATE_ERROR_WARNING;
		else
			can_state = CAN_STATE_ERROR_ACTIVE;

		/* error warning interrupt */
		netdev_dbg(netdev, "error warning interrupt %x, status: %d\n",
			   sr, can_state);
	}

	if (isrc & IRQ_BEI) {
		/* bus error interrupt */
		netdev_dbg(netdev, "bus error interrupt\n");

		priv->can.can_stats.bus_error++;
		stats->rx_errors++;

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		/* set error type */
		switch (ecc & ECC_MASK) {
		case ECC_BIT:
			netdev_dbg(netdev, "Bus Error: ECC_BIT %x\n", sr);
			cf->data[2] |= CAN_ERR_PROT_BIT;
			break;
		case ECC_FORM:
			netdev_dbg(netdev, "Bus Error: ECC_FORM %x\n", sr);
			cf->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case ECC_STUFF:
			netdev_dbg(netdev, "Bus Error: ECC_STUFF %x\n", sr);
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		default:
			netdev_dbg(netdev, "default %x\n", sr);
			break;
		}

		/* set error location */
		cf->data[3] = ecc & ECC_SEG;

		/* Error occurred during transmission? */
		if ((ecc & ECC_DIR) == 0) {
			cf->data[2] |= CAN_ERR_PROT_TX;
			netdev_dbg(netdev, "ecc tx %x\n", sr);
		} else {
			netdev_dbg(netdev, "ecc rx %x\n", sr);
		}
		spin_lock_irqsave(&priv->lock, flags);
		priv->need_clear_ecc = true;
		spin_unlock_irqrestore(&priv->lock, flags);

		schedule_work(&priv->handle_clear_reg_work);
	}

	if (isrc & IRQ_EPI) {
		if (can_state == CAN_STATE_ERROR_PASSIVE)
			can_state = CAN_STATE_ERROR_WARNING;
		else
			can_state = CAN_STATE_ERROR_PASSIVE;

		/* error passive interrupt */
		netdev_dbg(netdev, "error passive interrupt: %x\n", can_state);
	}

	if (isrc & IRQ_ALI) {
		/* arbitration lost interrupt */
		netdev_dbg(netdev, "arbitration lost interrupt\n");

		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = alc & 0x1f;

		spin_lock_irqsave(&priv->lock, flags);
		priv->need_clear_alc = true;
		spin_unlock_irqrestore(&priv->lock, flags);

		schedule_work(&priv->handle_clear_reg_work);
	}

	if (can_state != priv->can.state) {
		tx_state = txerr >= rxerr ? can_state : 0;
		rx_state = txerr <= rxerr ? can_state : 0;

		//netdev_info(netdev, "%s: %d\n", __func__, __LINE__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		can_change_state(netdev, cf, tx_state, rx_state);
#else
		if (can_state == CAN_STATE_ERROR_WARNING) {
			priv->can.can_stats.error_warning++;
			cf->data[1] = (txerr > rxerr) ?
					      CAN_ERR_CRTL_TX_WARNING :
					      CAN_ERR_CRTL_RX_WARNING;
		} else {
			priv->can.can_stats.error_passive++;
			cf->data[1] = (txerr > rxerr) ?
					      CAN_ERR_CRTL_TX_PASSIVE :
					      CAN_ERR_CRTL_RX_PASSIVE;
		}
#endif

		if (can_state == CAN_STATE_BUS_OFF) {
			//netdev_info(netdev, "%s: %d\n", __func__, __LINE__);
			//spin_lock_irqsave(&priv->lock, flags);

			//if (priv->can.state != CAN_STATE_BUS_OFF) {
			//netdev_info(netdev, "%s: 333 sr: %02x\n", __func__, data[0]);
			//priv->can.state = CAN_STATE_BUS_OFF;
			can_bus_off(netdev);
			//}

			//spin_unlock_irqrestore(&priv->lock, flags);
		}
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);

resubmit_urb:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status == -ENODEV) {
		netif_device_detach(netdev);
		netdev_err(netdev, "%s: nodev\n", __func__);
	} else if (status) {
		netdev_err(netdev,
			   "%s: failed resubmitting read bulk urb: %d\n",
			   __func__, status);
	}
}

/* Send data to device */
static netdev_tx_t f81604_start_xmit(struct sk_buff *skb,
				     struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct net_device_stats *stats = &netdev->stats;
	u8 *ptr;
	u32 id;
	int status;

	//netdev_info(netdev, "%s: in TX\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;
#endif

	netif_stop_queue(netdev);

	ptr = priv->bulk_write_buffer;
	memset(ptr, 0, F81604_DATA_SIZE);

	priv->tx_size = cf->can_dlc & 0xf;
	ptr[0] = F81604_CMD_DATA;
	ptr[1] = cf->can_dlc & 0xf;

	if (cf->can_id & CAN_EFF_FLAG) {
		id = (cf->can_id & CAN_ERR_MASK) << 3;
		ptr[1] |= F81604_EFF_BIT;
		ptr[2] = (id >> 24) & 0xff;
		ptr[3] = (id >> 16) & 0xff;
		ptr[4] = (id >> 8) & 0xff;
		ptr[5] = (id >> 0) & 0xff;
		memcpy(&ptr[6], cf->data, priv->tx_size);
	} else {
		id = (cf->can_id & CAN_ERR_MASK) << 5;
		ptr[2] = (id >> 8) & 0xff;
		ptr[3] = (id >> 0) & 0xff;
		memcpy(&ptr[4], cf->data, priv->tx_size);
	}

	if (cf->can_id & CAN_RTR_FLAG)
		ptr[1] |= F81604_RTR_BIT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	can_put_echo_skb(skb, netdev, 0, 0);
#else
	can_put_echo_skb(skb, netdev, 0);
#endif

	//netdev_info(netdev, "%s: TX submit, id: %08x\n", __func__, cf->can_id);

	status = usb_submit_urb(priv->write_urb, GFP_ATOMIC);
	if (status) {
		netdev_err(netdev,
			   "%s: failed resubmitting read bulk urb: %d\n",
			   __func__, status);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
		can_free_echo_skb(netdev, 0, NULL);
#else
		can_free_echo_skb(netdev, 0);
#endif
		dev_kfree_skb(skb);

		stats->tx_dropped++;

		if (status == -ENODEV)
			netif_device_detach(netdev);
	}

	return NETDEV_TX_OK;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
static int f81604_get_berr_counter(const struct net_device *netdev,
				   struct can_berr_counter *bec)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int status;
	u8 txerr;
	u8 rxerr;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_TXERR, &txerr);
	if (status)
		return status;

	status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_RXERR, &rxerr);
	if (status)
		return status;

	bec->txerr = txerr;
	bec->rxerr = rxerr;

	return 0;
}
#endif

static int f81604_prepare_urbs(struct net_device *netdev)
{
	static u8 bulk_in_addr[F81604_MAX_DEV] = { 0x82, 0x84 };
	static u8 bulk_out_addr[F81604_MAX_DEV] = { 0x01, 0x03 };
	static u8 int_in_addr[F81604_MAX_DEV] = { 0x81, 0x83 };
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int id = netdev->dev_id;
	int status;
	int i;

	for (i = 0; i < F81604_MAX_RX_URBS; ++i) {
		priv->read_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!priv->read_urb[i]) {
			status = -ENOMEM;
			goto error;
		}

		priv->bulk_read_buffer[i] =
			kzalloc(F81604_BULK_SIZE, GFP_KERNEL);
		if (!priv->bulk_read_buffer[i]) {
			status = -ENOMEM;
			goto error;
		}

		usb_fill_bulk_urb(priv->read_urb[i], priv->dev,
				  usb_rcvbulkpipe(priv->dev, bulk_in_addr[id]),
				  priv->bulk_read_buffer[i], F81604_BULK_SIZE,
				  f81604_read_bulk_callback, netdev);
	}

	priv->write_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!priv->write_urb) {
		status = -ENOMEM;
		goto error;
	}

	priv->bulk_write_buffer = kzalloc(F81604_DATA_SIZE, GFP_KERNEL);
	if (!priv->bulk_write_buffer) {
		status = -ENOMEM;
		goto error;
	}

	usb_fill_bulk_urb(priv->write_urb, priv->dev,
			  usb_sndbulkpipe(priv->dev, bulk_out_addr[id]),
			  priv->bulk_write_buffer, F81604_DATA_SIZE,
			  f81604_write_bulk_callback, netdev);

#if F81604_WRITE_READ_TEST
	priv->write_reg_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!priv->write_reg_urb) {
		status = -ENOMEM;
		goto error;
	}

	priv->bulk_write_reg_buffer =
		kzalloc(F81604_READ_URB_SIZE, GFP_KERNEL);
	if (!priv->bulk_write_reg_buffer) {
		status = -ENOMEM;
		goto error;
	}

	usb_fill_bulk_urb(priv->write_reg_urb, priv->dev,
			  usb_sndbulkpipe(priv->dev, bulk_out_addr[id]),
			  priv->bulk_write_reg_buffer, F81604_READ_URB_SIZE,
			  f81604_write_reg_bulk_callback, netdev);

	priv->bulk_write_reg_buffer[0] = F81604_CMD_WRITE_REG;
	priv->bulk_write_reg_buffer[1] = F81604_READ_REG_OFFSET;
	priv->bulk_write_reg_buffer[2] = F81604_READ_REG_URB_SIZE - 1;
#endif

	for (i = 0; i < F81604_MAX_RX_REG_URBS; ++i) {
		priv->read_sr_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!priv->read_sr_urb[i]) {
			status = -ENOMEM;
			goto error;
		}

		priv->bulk_read_sr_buffer[i] =
			kzalloc(F81604_READ_URB_SIZE, GFP_KERNEL);
		if (!priv->bulk_read_sr_buffer[i]) {
			status = -ENOMEM;
			goto error;
		}

		usb_fill_bulk_urb(
			priv->read_sr_urb[i], priv->dev,
			usb_sndbulkpipe(priv->dev, bulk_out_addr[id]),
			priv->bulk_read_sr_buffer[i], F81604_READ_URB_SIZE,
			f81604_read_sr_bulk_callback, netdev);
	}

	priv->int_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!priv->int_urb) {
		status = -ENOMEM;
		goto error;
	}

	priv->int_read_buffer = kzalloc(F81604_INT_SIZE, GFP_KERNEL);
	if (!priv->int_read_buffer) {
		status = -ENOMEM;
		goto error;
	}

	usb_fill_int_urb(priv->int_urb, priv->dev,
			 usb_rcvintpipe(priv->dev, int_in_addr[id]),
			 priv->int_read_buffer, F81604_INT_SIZE,
			 f81604_read_int_callback, netdev, 1);

	return 0;

error:
	/* todo error */
	return status;
}

static void f81604_remove_urbs(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int i;

	for (i = 0; i < F81604_MAX_RX_URBS; ++i) {
		usb_free_urb(priv->read_urb[i]);
		kfree(priv->bulk_read_buffer[i]);
	}

	usb_free_urb(priv->write_urb);
	kfree(priv->bulk_write_buffer);

	for (i = 0; i < F81604_MAX_RX_REG_URBS; ++i) {
		usb_free_urb(priv->read_sr_urb[i]);
		kfree(priv->bulk_read_sr_buffer[i]);
	}

	usb_free_urb(priv->int_urb);
	kfree(priv->int_read_buffer);

#if F81604_WRITE_READ_TEST
	usb_free_urb(priv->write_reg_urb);
	kfree(priv->bulk_write_reg_buffer);
#endif
}

static int f81604_register_urbs(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int status;
	int i;

	for (i = 0; i < F81604_MAX_RX_URBS; ++i) {
		status = usb_submit_urb(priv->read_urb[i], GFP_KERNEL);
		if (status) {
			pr_err("%s: submit read urb failed: %d\n", __func__,
			       status);
			return status;
		}
	}

#if !F81604_POLL_MODE
	status = usb_submit_urb(priv->int_urb, GFP_KERNEL);
	if (status) {
		pr_err("%s: submit int urb failed: %d\n", __func__, status);
		return status;
	}
#endif

#if F81604_WRITE_READ_TEST
	status = usb_submit_urb(priv->write_reg_urb, GFP_KERNEL);
	if (status) {
		pr_err("%s: submit write_reg urb failed: %d\n", __func__,
		       status);
		return status;
	}
#endif

	return 0;
}

static void f81604_unregister_urbs(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	int i;

	usb_kill_urb(priv->write_urb);

	for (i = 0; i < F81604_MAX_RX_REG_URBS; ++i)
		usb_kill_urb(priv->read_sr_urb[i]);

	for (i = 0; i < F81604_MAX_RX_URBS; ++i)
		usb_kill_urb(priv->read_urb[i]);

	usb_kill_urb(priv->int_urb);

#if F81604_WRITE_READ_TEST
	usb_kill_urb(priv->write_reg_urb);
#endif

	for (i = 0; i < F81604_MAX_RX_REG_URBS; ++i)
		clear_bit(i, &priv->read_sr_flag);
}

/* Open USB device */
static int f81604_open(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);
	unsigned long flags;
	int err;

	spin_lock_irqsave(&priv->lock, flags);
	priv->is_txing = false;
	spin_unlock_irqrestore(&priv->lock, flags);

	err = f81604_prepare_urbs(netdev);
	if (err) {
		/* todo error */
		return err;
	}

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

#if !DIABLE_CANLED && LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
	can_led_event(netdev, CAN_LED_EVENT_OPEN);
#endif
	/* finally start device */
	err = f81604_start(netdev);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(netdev);

		netdev_warn(netdev, "couldn't start device: %d\n", err);

		close_candev(netdev);

		return err;
	}

	netif_start_queue(netdev);

#if F81604_DUMP_REG
	schedule_delayed_work(&priv->dump_reg_delayed_work, 1);
#endif

#if F81604_POLL_MODE
	schedule_delayed_work(&priv->poll_delayed_work, 1);
#endif
	return 0;
}

/* Close USB device */
static int f81604_close(struct net_device *netdev)
{
	struct f81604_port_priv *priv = netdev_priv(netdev);

	priv->can.state = CAN_STATE_STOPPED;
	f81604_set_reset_mode(netdev);

	netif_stop_queue(netdev);

	f81604_unregister_urbs(netdev);
	f81604_remove_urbs(netdev);

#if F81604_POLL_MODE
	cancel_work_sync(&priv->handle_int_work);
#endif
	cancel_work_sync(&priv->handle_clear_reg_work);

	close_candev(netdev);
#if !DIABLE_CANLED && LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
	can_led_event(netdev, CAN_LED_EVENT_STOP);
#endif

#if F81604_DUMP_REG
	cancel_delayed_work_sync(&priv->dump_reg_delayed_work);
#endif

#if F81604_POLL_MODE
	cancel_delayed_work_sync(&priv->poll_delayed_work);
#endif

	return 0;
}

static int f81604_ioctl_write(struct net_device *netdev, struct ifreq *ifr)
{
	struct f81604_reg_ctrl reg_ctrl;
	struct f81604_port_priv *priv;

	pr_debug("%s: %d\n", __func__, __LINE__);

	priv = netdev_priv(netdev);

	if (copy_from_user(&reg_ctrl, ifr->ifr_ifru.ifru_data,
			   sizeof(reg_ctrl)))
		return -EFAULT;

	switch (reg_ctrl.reg) {
	case 0x105:
	case 0x400 ... 0x655:
		break;

	default:
		return -EINVAL;
	}

	pr_debug("%s: %x %x\n", __func__, reg_ctrl.reg, reg_ctrl.data);

	return f81604_set_register(priv->dev, reg_ctrl.reg, reg_ctrl.data);
}

static int f81604_ioctl_read(struct net_device *netdev, struct ifreq *ifr)
{
	struct f81604_reg_ctrl reg_ctrl;
	struct f81604_port_priv *priv;
	int r;

	pr_debug("%s: %d\n", __func__, __LINE__);

	priv = netdev_priv(netdev);

	if (copy_from_user(&reg_ctrl, ifr->ifr_ifru.ifru_data,
			   sizeof(reg_ctrl)))
		return -EFAULT;

	switch (reg_ctrl.reg) {
	case 0x105:
	case 0x400 ... 0x655:
		break;

	default:
		return -EINVAL;
	}

	r = f81604_get_register(priv->dev, reg_ctrl.reg, &reg_ctrl.data);
	if (r)
		return r;

	if (copy_to_user((int __user *)ifr->ifr_ifru.ifru_data, &reg_ctrl,
			 sizeof(reg_ctrl)))
		return -EFAULT;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
static int f81604_ndo_siocdevprivate(struct net_device *dev, struct ifreq *ifr,
				     void __user *data, int cmd)
{
	switch (cmd) {
	case FINTEK_IOCTL_WR_REG:
		return f81604_ioctl_write(dev, ifr);

	case FINTEK_IOCTL_RD_REG:
		return f81604_ioctl_read(dev, ifr);
	}

	return -EOPNOTSUPP;
}
#else
static int f81604_ndo_do_ioctl(struct net_device *dev, struct ifreq *ifr,
			       int cmd)
{
	switch (cmd) {
	case FINTEK_IOCTL_WR_REG:
		return f81604_ioctl_write(dev, ifr);

	case FINTEK_IOCTL_RD_REG:
		return f81604_ioctl_read(dev, ifr);
	}

	return -ENOTSUPP;
}
#endif

static const struct net_device_ops f81604_netdev_ops = {
	.ndo_open = f81604_open,
	.ndo_stop = f81604_close,
	.ndo_start_xmit = f81604_start_xmit,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	.ndo_change_mtu = can_change_mtu,
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	.ndo_siocdevprivate = f81604_ndo_siocdevprivate,
#else
	.ndo_do_ioctl = f81604_ndo_do_ioctl,
#endif

};

static const struct can_bittiming_const f81604_bittiming_const = {
	.name = "f81604",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

#if 0
void f81604_test(struct usb_device *dev)
{
	int status;
	int reg;
	u8 tmp;

	reg = 0x1000;
	status = f81604_get_register(dev, reg, &tmp);
	pr_info("%s: %04x %x, %x\n", __func__, reg, status, tmp);

	reg = 0x2000;
	status = f81604_get_register(dev, reg, &tmp);
	pr_info("%s: %04x %x, %x\n", __func__, reg, status, tmp);
}
#endif

#if F81604_DUMP_REG
static void f81604_dump_reg_delayed_work(struct work_struct *work)
{
	struct f81604_port_priv *priv = container_of(
		work, struct f81604_port_priv, dump_reg_delayed_work.work);
	struct net_device *netdev = priv->can.dev;
	int status;
	u8 tmp;
	int count = 5;

	while (count--) {
#if 1
		status = f81604_get_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_IR, &tmp);
		if (!status) {
			priv->ali = !!(tmp & IRQ_ALI);

			if (priv->ali) {
				status = f81604_get_sja1000_register(
					priv->dev, netdev->dev_id,
					SJA1000_IR - 1, &tmp);

				status = f81604_get_sja1000_register(
					priv->dev, netdev->dev_id, SJA1000_IR,
					&tmp);
				if (tmp & IRQ_ALI) {
					pr_info("%s: cant clear\n", __func__);
				}

				status = f81604_get_sja1000_register(
					priv->dev, netdev->dev_id,
					SJA1000_ALC - 1, &tmp);

				status = f81604_get_sja1000_register(
					priv->dev, netdev->dev_id, SJA1000_ALC,
					&tmp);
				if (!status) {
					if (priv->alc != tmp) {
						pr_info("dev_id: %d, ALC: %x\n",
							netdev->dev_id, tmp);
						priv->alc = tmp;
					}
				}
			} else {
				//pr_info("dev: %d IRQ_ALI: %d\n", netdev->dev_id, priv->ali);
			}
		}
#endif

		if (status)
			break;
	}

	//msecs_to_jiffies
	schedule_delayed_work(&priv->dump_reg_delayed_work, 1);
}

#endif

#if F81604_POLL_MODE
static void f81604_poll_delayed_work(struct work_struct *work)
{
	struct f81604_port_priv *priv = container_of(
		work, struct f81604_port_priv, poll_delayed_work.work);

	f81604_int_handle_work(&priv->handle_int_work);
	schedule_delayed_work(&priv->poll_delayed_work, 1);
}
#endif

#if F81604_MONITOR_WRITE_BULK
static void f81604_write_bulk_monitor_delayed_work(struct work_struct *work)
{
	struct f81604_port_priv *priv =
		container_of(work, struct f81604_port_priv,
			     write_bulk_monitor_delayed_work.work);

	netdev_info(priv->netdev, "%s: wakeup\n", __func__);

	usb_kill_urb(priv->read_sr_urb[priv->current_sr_idx]);
	f81604_register_read_sr_urbs(priv->netdev);
}
#endif

static int f81604_can_pre_init(struct usb_interface *intf, int can_id)
{
	struct usb_device *dev = interface_to_usbdev(intf);
	unsigned char tmp, data;
	unsigned char key[4] = { 0x32, 0x5d, 0x42, 0xac };
	int i, status;

	status = f81604_set_sja1000_register(dev, can_id, 0x7c,
					     0x35); // force deactive
	if (status)
		return status;

	for (i = 0; i < ARRAY_SIZE(key); ++i) {
		status =
			f81604_set_sja1000_register(dev, can_id, 0x7c, key[i]);
		if (status)
			return status;
	}

	data = BIT(5) | BIT(4);

	status = f81604_set_sja1000_register(dev, can_id, 0x7f, data);
	if (status)
		return status;

	status = f81604_get_sja1000_register(dev, can_id, 0x7f, &tmp);
	if (status)
		return status;

	return f81604_set_sja1000_register(dev, can_id, 0x7c,
					   0x35); // force deactive
}

static int f81604_is_hw_disable(struct usb_interface *intf, int ch)
{
	struct usb_device *dev = interface_to_usbdev(intf);
	int status, i, r = 0, max_check = 100;
	u8 sr;

	if (!hwdisable_detect)
		return 0;

	/* set clock divider and output control register */
	status = f81604_set_sja1000_register(dev, ch, SJA1000_CDR,
					     CDR_CBP | CDR_PELICAN);
	if (status)
		return status;

	status = f81604_set_sja1000_register(
		dev, ch, SJA1000_OCR,
		OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL | OCR_MODE_NORMAL);
	if (status)
		return status;

	for (i = 0; i < max_check; ++i) {
		f81604_set_sja1000_register(dev, ch, SJA1000_MOD, MOD_RM);

		f81604_set_sja1000_register(dev, ch, SJA1000_IER, 0x00);
		f81604_set_sja1000_register(dev, ch, SJA1000_BTR0, 0xc0);
		f81604_set_sja1000_register(dev, ch, SJA1000_BTR1, 0x27);

		status = f81604_set_sja1000_register(dev, ch, SJA1000_MOD,
						     MOD_LOM);
		if (status)
			break;

		usleep_range(100, 100);

		status = f81604_get_sja1000_register(dev, ch, SJA1000_SR, &sr);
		if (status)
			break;

		if ((sr == 0x3c) && ((sr & SR_ES) != SR_ES))
			r += 1;
	}

	f81604_set_sja1000_register(dev, ch, SJA1000_MOD, MOD_RM);
	f81604_set_sja1000_register(dev, ch, SJA1000_CDR, 0);

	if (r >= max_check * 9 / 10) {
		pr_info("%s: ch: %d, i: %d, r: %d\n", __func__, ch, i, r);
		return 1;
	}

	return 0;
}

/* Probe USB device
 *
 * Check device and firmware.
 * Set supported modes and bittiming constants.
 * Allocate some memory.
 */
static int f81604_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(intf);
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *epd;
	struct f81604_priv *priv;
	struct f81604_port_priv *port_priv;
	int i, err;
	int out_ep_count = 0;
	u8 speed[2];
	u8 ver;

	priv = devm_kzalloc(&intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	usb_set_intfdata(intf, priv);
	mutex_init(&priv->mutex);

#if F81604_MONITOR_WRITE_BULK
	pr_info("%s: write_bulk_timeout: %d\n", __func__, write_bulk_timeout);
#endif

#if F81604_INT_WAKE_TX
	pr_info("%s: int_wake_tx: %d\n", __func__, int_wake_tx);
#endif

	pr_info("%s: Fintek F81604 driver version: %s\n", __func__, DRV_VER);

	iface_desc = intf->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		epd = &iface_desc->endpoint[i].desc;
		if (usb_endpoint_is_bulk_out(epd))
			out_ep_count++;
	}

	err = f81604_get_sja1000_register(dev, 0, 9, &speed[0]);
	if (err)
		return err;

	err = f81604_get_sja1000_register(dev, 1, 9, &speed[1]);
	if (err)
		return err;

	err = f81604_get_register(dev, F81604_HW_VERSION, &ver);
	if (err)
		return err;

	for (i = 0; i < F81604_MAX_DEV && i < out_ep_count; ++i) {
		f81604_can_pre_init(intf, i);

#if F81604_HWDISABLE
		err = f81604_is_hw_disable(intf, i);
		if (err) {
			dev_info(&intf->dev, "%s: channel %d is h/w diabled\n",
				 __func__, i);
			continue;
		}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
		priv->netdev[i] = alloc_candev(sizeof(*port_priv), 1);
#else
		priv->netdev[i] = alloc_candev(sizeof(*port_priv));
#endif
		if (!priv->netdev[i]) {
			dev_err(&intf->dev, "Couldn't alloc candev: %d\n", i);
			return -ENOMEM;
		}

		port_priv = netdev_priv(priv->netdev[i]);
		priv->netdev[i]->dev_id = i;

		port_priv->is_txing = 0;
		spin_lock_init(&port_priv->lock);
		INIT_WORK(&port_priv->handle_clear_overrun_work,
			  f81604_handle_clear_overrun_work);
		INIT_WORK(&port_priv->handle_clear_reg_work,
			  f81604_handle_clear_reg_work);
#if F81604_POLL_MODE
		INIT_WORK(&port_priv->handle_int_work, f81604_int_handle_work);
#endif

#if F81604_DUMP_REG
		INIT_DELAYED_WORK(&port_priv->dump_reg_delayed_work,
				  f81604_dump_reg_delayed_work);
#endif

#if F81604_POLL_MODE
		INIT_DELAYED_WORK(&port_priv->poll_delayed_work,
				  f81604_poll_delayed_work);
#endif
#if F81604_MONITOR_WRITE_BULK
		INIT_DELAYED_WORK(&port_priv->write_bulk_monitor_delayed_work,
				  f81604_write_bulk_monitor_delayed_work);
#endif
		port_priv->intf = intf;
		port_priv->dev = dev;
		port_priv->ver = ver;
		port_priv->ocr = OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL;
		port_priv->cdr = CDR_CBP;
		port_priv->tx_size = 0;
		port_priv->can.state = CAN_STATE_STOPPED;
		port_priv->can.clock.freq = F81604_V1_CLOCK;

		if (speed[i] == 1)
			port_priv->can.clock.freq = F81604_V2_CLOCK;

		port_priv->can.bittiming_const = &f81604_bittiming_const;
		port_priv->can.do_set_bittiming = f81604_set_bittiming;
		port_priv->can.do_set_mode = f81604_set_mode;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		port_priv->can.do_get_berr_counter = f81604_get_berr_counter;
		port_priv->can.ctrlmode_supported = /*CAN_CTRLMODE_LOOPBACK |*/
			CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_3_SAMPLES |
			CAN_CTRLMODE_ONE_SHOT | CAN_CTRLMODE_BERR_REPORTING;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
		port_priv->can.ctrlmode_supported |= CAN_CTRLMODE_PRESUME_ACK;
#endif
		priv->netdev[i]->netdev_ops = &f81604_netdev_ops;
		priv->netdev[i]->flags |= IFF_ECHO; /* we support local echo */

		SET_NETDEV_DEV(priv->netdev[i], &intf->dev);

		err = register_candev(priv->netdev[i]);
		if (err) {
			netdev_err(priv->netdev[i],
				   "couldn't register CAN device: %d\n", err);
			return err;
		}

		port_priv->netdev = priv->netdev[i];

		dev_info(&intf->dev, "Channel #%d registered as %s\n", i,
			 priv->netdev[i]->name);

		device_create_file(&priv->netdev[i]->dev,
				   &dev_attr_terminator_control);

		device_create_file(&priv->netdev[i]->dev,
				   &dev_attr_force_read);
		device_create_file(&priv->netdev[i]->dev, &dev_attr_force_tx);
#if !DIABLE_CANLED && LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
		devm_can_led_init(priv->netdev[i]);
#endif
	}

	/* todo error recovery */
	return 0;
}

/* Called by the usb core when driver is unloaded or device is removed */
static void f81604_disconnect(struct usb_interface *intf)
{
	struct f81604_priv *priv = usb_get_intfdata(intf);
	int i;

	for (i = 0; i < F81604_MAX_DEV; ++i) {
		if (!priv->netdev[i])
			continue;

		netdev_info(priv->netdev[i], "device disconnected\n");

		device_remove_file(&priv->netdev[i]->dev,
				   &dev_attr_terminator_control);

		device_remove_file(&priv->netdev[i]->dev,
				   &dev_attr_force_read);
		device_remove_file(&priv->netdev[i]->dev, &dev_attr_force_tx);

		unregister_netdev(priv->netdev[i]);
		free_candev(priv->netdev[i]);
	}
}

static int f81604_suspend(struct usb_interface *intf, pm_message_t message)
{
	dev_info(&intf->dev, "%s\n", __func__);
	return 0;
}

static int f81604_resume(struct usb_interface *intf)
{
	dev_info(&intf->dev, "%s\n", __func__);
	return 0;
}

static struct usb_driver f81604_driver = {
	.name = "f81604",
	.probe = f81604_probe,
	.disconnect = f81604_disconnect,
	.suspend = f81604_suspend,
	.resume = f81604_resume,
	.id_table = f81604_table,
};

module_usb_driver(f81604_driver);

MODULE_AUTHOR("Peter Hong <Peter_Hong@fintek.com.tw>");
MODULE_DESCRIPTION("Fintek F81603/604 USB to CANBUS");
MODULE_LICENSE("GPL v2");
