/*
 * CAN driver for "Fintek F81605" USB-to-1/2CANFD converter
 *
 * Copyright (C) 2025 Ji-Ze Hong (Peter Hong) (hpeter+linux_kernel@gmail.com)
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
 */

#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/ktime.h>
#include <linux/reboot.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#include <linux/minmax.h>
#endif

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
#include <linux/can/led.h>
#endif

#include "sja1000.h"

#ifndef CAN_CTRLMODE_BERR_REPORTING
#define CAN_CTRLMODE_BERR_REPORTING 0x10 /* Bus-error reporting */
#define CAN_CTRLMODE_ONE_SHOT 0x08 /* One-Shot mode */
#endif

#define DRV_VER "v1.05-v20260226"
#define DRV_NAME "F81605"

#define F81605_EXTEND_BTR 1

#define CAN_BASE_ADDRESS 0x600
#define CAN_TIMER_DBG 0
#define CAN_DELAYED_RX_DBG 0

#define CAN_STS_POLL 1
#define CAN_STS_POLL_TIMER msecs_to_jiffies(1)

/* vendor and product id */
#define F81605_VENDOR_ID 0x2c42
#define F81605_PRODUCT_ID 0x2104

#define F81605_USB_MAX_RETRY 10
#define F81605_USB_TIMEOUT 2000
#define F81605_SET_GET_REGISTER 0xA0
#define F81605_PORT_OFFSET 0x100

#define F81605_HW_VERSION 0x10f
#define F81605_GPIO_MODE_REG 0x106
#define F81605_GPIO_DATA_REG 0x107

#define F81605_MAX_DEV 2

#if CAN_DELAYED_RX_DBG
#define F81605_MAX_RX_URBS 1
#else
#define F81605_MAX_RX_URBS 8
#endif

#define F81605_MAX_TX_URBS 2

#define F81605_RX_BULK_SIZE 512
#define F81605_TX_FRAME_SIZE 512

//#define SJA1000_FI_RTR 0x40
#define F81605_FI_EDL BIT(5)
#define F81605_FI_BRS BIT(4)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
#define can_dlc2len can_fd_dlc2len
#define can_len2dlc can_fd_len2dlc
#endif

static unsigned int disable_canfd = 0;
module_param(disable_canfd, uint, S_IRUGO);
MODULE_PARM_DESC(disable_canfd, "disable_canfd");

static unsigned int force_sjw_max = 1;
module_param(force_sjw_max, uint, S_IRUGO);
MODULE_PARM_DESC(force_sjw_max, "force_sjw_max");

static int disable_ssp = -1;
module_param(disable_ssp, int, S_IRUGO);
MODULE_PARM_DESC(disable_ssp,
		 "Disable TX delay ACKed, 1=disable, 0=enable, -1=auto");

static int ssp_value = -1; //-1
module_param(ssp_value, int, S_IRUGO);
MODULE_PARM_DESC(ssp_value, "SSP value, must >= 2, -1=auto");

static unsigned int auto_ssp_offset = 1;
module_param(auto_ssp_offset, uint, S_IRUGO);
MODULE_PARM_DESC(auto_ssp_offset, "auto_ssp_offset");

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "debug");

static unsigned int reg_debug = 0;
module_param(reg_debug, uint, S_IRUGO);
MODULE_PARM_DESC(reg_debug, "reg_debug");

struct f81605_reg_ctrl {
	unsigned int reg;
	unsigned char data;
};

#define FINTEK_IOCTL_WR_REG (SIOCDEVPRIVATE + 0)
#define FINTEK_IOCTL_RD_REG (SIOCDEVPRIVATE + 1)

/* table of devices that work with this driver */
static const struct usb_device_id F81605_table[] = {
	{ USB_DEVICE(F81605_VENDOR_ID, F81605_PRODUCT_ID) },
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, F81605_table);

struct f81605_priv {
	int dev_count;
	struct net_device *netdev[F81605_MAX_DEV];
	struct mutex mutex;

	struct notifier_block reboot_notifier;
	struct usb_interface *intf;
};

struct f81605_port_priv {
	struct can_priv can;
	struct net_device *netdev;
	struct sk_buff *echo_skb;

	spinlock_t lock;
	struct workqueue_struct *wq;

#if CAN_TIMER_DBG
	struct delayed_work delayed_timer_work;
#endif

#if CAN_DELAYED_RX_DBG
	struct delayed_work delayed_rx_work;
#endif

#if CAN_STS_POLL
	struct delayed_work delayed_sts_work;
#endif

	struct delayed_work delayed_oneshot_work;

	struct usb_device *dev;
	struct usb_interface *intf;

	struct urb *read_rx_urb[F81605_MAX_RX_URBS];
	u8 bulk_read_rx_buffer[F81605_MAX_RX_URBS][F81605_RX_BULK_SIZE];

	struct urb *read_sts_urb[F81605_MAX_RX_URBS];
	u8 bulk_read_sts_buffer[F81605_MAX_RX_URBS][F81605_RX_BULK_SIZE];

	struct urb *write_urb[F81605_MAX_TX_URBS];
	u8 bulk_write_buffer[F81605_MAX_TX_URBS][F81605_RX_BULK_SIZE];
	unsigned long write_urb_sts;

	u8 ocr;
	u8 cdr;
	u8 old_mod, old_oneshot_tec;

#if CAN_TIMER_DBG
	u8 auto_ssp;
#endif

	int ver;
	bool is_disable_fd;
	bool is_tx_callbacked;
};

static const struct can_bittiming_const f81605_bittiming_const = {
	.name = DRV_NAME,

	.tseg1_min = 1,
	.tseg2_min = 1,
	.brp_min = 1,
	.brp_inc = 1,

#if F81605_EXTEND_BTR
	.tseg1_max = 1 << 8,
	.tseg2_max = 1 << 7,
	.sjw_max = 1 << 7,
	.brp_max = 1 << 14,

#else
	.tseg1_max = 16,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_max = 64,
#endif
};

static const struct can_bittiming_const f81605_data_bittiming_const = {
	.name = DRV_NAME,

	.tseg1_min = 1,
	.tseg2_min = 1,
	.brp_min = 1,
	.brp_inc = 1,

#if F81605_EXTEND_BTR
	.tseg1_max = 1 << 8,
	.tseg2_max = 1 << 7,
	.sjw_max = 1 << 7,
	.brp_max = 64,
#else
	.tseg1_max = 16,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_max = 64,
#endif
};

#ifndef get_can_dlc
#define get_can_dlc can_cc_dlc2len
#endif

static void f81605_unregister_urbs(struct net_device *netdev);
static int f81605_register_urbs(struct net_device *netdev);

static int f81605_set_register(struct usb_device *dev, u16 reg, u8 data)
{
	size_t count = F81605_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	*tmp = data;

	while (count--) {
		status = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
					 F81605_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_OUT, 0, reg,
					 tmp, sizeof(u8), F81605_USB_TIMEOUT);
		if (status > 0) {
			status = 0;
			break;
		} else if (status == 0) {
			status = -EIO;
		}
	}

	if (status < 0) {
		dev_dbg(&dev->dev, "%s: reg: %x data: %x failed: %d\n",
			__func__, reg, data, status);
	}

	kfree(tmp);
	return status;
}

static int f81605_get_register(struct usb_device *dev, u16 reg, u8 *data)
{
	size_t count = F81605_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	while (count--) {
		status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
					 F81605_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_IN, 0, reg,
					 tmp, sizeof(u8), F81605_USB_TIMEOUT);
		if (status > 0) {
			status = 0;
			break;
		} else if (status == 0) {
			status = -EIO;
		}
	}

	if (status < 0) {
		dev_dbg(&dev->dev, "%s: reg: %x failed: %d\n", __func__, reg,
			status);
		goto end;
	}

	*data = *tmp;

end:
	kfree(tmp);
	return status;
}

static int f81605_set_mask_register(struct usb_device *dev, u16 reg, u8 mask,
				    u8 data)

{
	int status;
	u8 tmp;

	status = f81605_get_register(dev, reg, &tmp);
	if (status)
		return status;

	tmp &= ~mask;
	tmp |= (mask & data);

	return f81605_set_register(dev, reg, tmp);
}

static int f81605_set_sja1000_register(struct usb_device *dev, u8 port,
				       u16 reg, u8 data)
{
	return f81605_set_register(
		dev, reg + F81605_PORT_OFFSET * port + CAN_BASE_ADDRESS, data);
}

static int f81605_get_sja1000_register(struct usb_device *dev, u8 port,
				       u16 reg, u8 *data)
{
	return f81605_get_register(
		dev, reg + F81605_PORT_OFFSET * port + CAN_BASE_ADDRESS, data);
}

static int f81605_set_mask_sja1000_register(struct usb_device *dev, u8 port,
					    u16 reg, u8 mask, u8 data)

{
	int status;
	u8 tmp;

	status = f81605_get_sja1000_register(dev, port, reg, &tmp);
	if (status)
		return status;

	tmp &= ~mask;
	tmp |= (mask & data);

	return f81605_set_sja1000_register(dev, port, reg, tmp);
}

static int f81605_set_reset_mode(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	u8 tmp;
	int status;
	int i;

#if CAN_STS_POLL
	cancel_delayed_work_sync(&priv->delayed_sts_work);
#endif

	/* disable interrupts */
	status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_IER, IRQ_OFF);
	if (status)
		return status;

	for (i = 0; i < 100; i++) {
		status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, &tmp);
		if (status)
			return status;

		/* check reset bit */
		if (tmp & MOD_RM) {
			priv->can.state = CAN_STATE_STOPPED;
			return 0;
		}

		/* reset chip */
		status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, MOD_RM);
		if (status)
			return status;
	}

	netdev_err(netdev, "setting SJA1000 into reset mode failed!\n");
	return -EINVAL;
}

static int f81605_set_normal_mode(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int status, i;
	u8 tmp;
	u8 mod_reg_val = 0x00;
	u8 ier = 0;

	for (i = 0; i < 100; i++) {
		status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_MOD, &tmp);
		if (status)
			return status;

		/* check reset bit */
		if ((tmp & MOD_RM) == 0) {
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
			/* enable interrupts */

#if 0
			if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) {
				ier = IRQ_ALL & ~(IRQ_TI | IRQ_RI);

				if (!more_err_report)
					ier &= ~(IRQ_BEI | IRQ_ALI);

				ier &= ~IRQ_ALI;
				status = f81605_set_sja1000_register(
					priv->dev, netdev->dev_id, SJA1000_IER,
					ier);
				if (status)
					return status;
			} else {
				ier = IRQ_ALL & ~(IRQ_TI | IRQ_RI | IRQ_BEI);

				if (!more_err_report)
					ier &= ~(IRQ_BEI | IRQ_ALI);

				ier &= ~IRQ_ALI;
				status = f81605_set_sja1000_register(
					priv->dev, netdev->dev_id, SJA1000_IER,
					ier);
				if (status)
					return status;
			}
#else
			ier = IRQ_ALL & ~(IRQ_TI | IRQ_RI | IRQ_ALI);

			status = f81605_set_sja1000_register(
				priv->dev, netdev->dev_id, SJA1000_IER, ier);
			if (status)
				return status;

#if CAN_STS_POLL
			queue_delayed_work(priv->wq, &priv->delayed_sts_work,
					   CAN_STS_POLL_TIMER);
#endif

#endif
			return 0;
		}

		/* set chip to normal mode */
		if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
			mod_reg_val |= MOD_LOM;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
		if ((priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) ||
		    (priv->can.ctrlmode & CAN_CTRLMODE_PRESUME_ACK))
			mod_reg_val |= MOD_STM;
#endif
		status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
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
static int f81605_chipset_init(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int status;
	int i;

	/* set clock divider and output control register */
	status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_CDR,
					     priv->cdr | CDR_PELICAN);
	if (status)
		return status;

	/* set acceptance filter (accept all) */
	for (i = 0; i < 4; ++i) {
		status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_ACCC0 + i, 0);
		if (status)
			return status;
	}

	for (i = 0; i < 4; ++i) {
		status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_ACCM0 + i, 0xFF);
		if (status)
			return status;
	}

	return f81605_set_sja1000_register(priv->dev, netdev->dev_id,
					   SJA1000_OCR,
					   priv->ocr | OCR_MODE_NORMAL);
}

static int f81605_start(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int status;
	u8 tmp;

	priv->is_tx_callbacked = false;
	priv->old_oneshot_tec = 0;

	f81605_unregister_urbs(netdev);

	status = f81605_set_mask_register(priv->dev, 0x108 + netdev->dev_id,
					  BIT(5) | BIT(0), BIT(5) | BIT(0));
	if (status)
		return status;

	/* set reset mode */
	status = f81605_set_reset_mode(netdev);
	if (status)
		return status;

	/* Initialize chip if uninitialized at this stage */
	status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_CDR, &tmp);
	if (status)
		return status;

	status = f81605_chipset_init(netdev);
	if (status)
		return status;

	/* Clear error counters and error code capture */
	status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_TXERR, 0);
	if (status)
		return status;

	status = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_RXERR, 0);
	if (status)
		return status;

	status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_ECC, &tmp);
	if (status)
		return status;

	status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_ALC, &tmp);
	if (status)
		return status;

	status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_IR, &tmp);
	if (status)
		return status;

	status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_SR, &tmp);
	if (status)
		return status;

	// interframe issue
	status = f81605_set_mask_sja1000_register(priv->dev, netdev->dev_id,
						  0x72, BIT(4), BIT(4));
	if (status)
		return status;

	// enable loading register
	status = f81605_set_mask_sja1000_register(priv->dev, netdev->dev_id,
						  0x75, BIT(0), BIT(0));
	if (status)
		return status;

	// if SRR, disable SSP
	if ((priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) ||
	    (priv->can.ctrlmode & CAN_CTRLMODE_PRESUME_ACK)) {
		status = f81605_set_mask_sja1000_register(
			priv->dev, netdev->dev_id, 0x77,
			BIT(5) | BIT(2) | BIT(0), 0);
		if (status)
			return status;
	}

	f81605_register_urbs(netdev);

#if CAN_TIMER_DBG
	schedule_delayed_work(&priv->delayed_timer_work,
			      msecs_to_jiffies(1000));
#endif

	/* leave reset mode */
	status = f81605_set_normal_mode(netdev);
	if (status)
		return status;

	return 0;
}

static int f81605_set_bittiming(struct net_device *dev)
{
	struct f81605_port_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u32 btr0, btr1, btr2, btr3, brp, seg1, seg2, sjw;
	int r = 0;

	brp = bt->brp - 1;
	sjw = bt->sjw - 1;
	seg1 = bt->prop_seg + bt->phase_seg1 - 1;
	seg2 = bt->phase_seg2 - 1;

	if (force_sjw_max)
		sjw = min(seg2, f81605_bittiming_const.sjw_max);

	btr0 = (brp & 0x3f) | ((sjw & 0x3) << 6);
	btr1 = (seg1 & 0xf) | ((seg2 & 0x7) << 4);

	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		btr1 |= 0x80;

	netdev_info(dev, "BTR0=0x%02x BTR1=0x%02x\n", btr0, btr1);

	r |= f81605_set_sja1000_register(priv->dev, dev->dev_id, SJA1000_BTR0,
					 btr0);
	r |= f81605_set_sja1000_register(priv->dev, dev->dev_id, SJA1000_BTR1,
					 btr1);

	btr2 = btr3 = 0;

#if F81605_EXTEND_BTR
	/*
	 	Index 0x05 (BTR2 : Bus Timing Register 2)		 
	 	Bit Name	 Description
	 	[7:0]	 brp_h	 brp = {brp_h[7:0], brp_l[5:0]}
	 */

	btr2 = brp >> 6;

	/*
		Index 0x09 (BTR3: Bus Timing Register3 )		
		Bit Name	Description
		[7:4]	tseg_2[6:3] tseg_2 = {tseg_2[6:3], tseg_2[2:0]}
		[3:0]	tseg_1[7:4] tseg_1 = {tseg_1[7:4], tseg_1[3:0]}

	*/

	btr3 = (GENMASK(3, 0) & (seg1 >> 4)) | (GENMASK(7, 4) & (seg2 << 1));

	/*
	 	Index 0x19 (BTR6 : Bus Timing Register 6)			 
	 	Bit Name	 Description default
	 	[7:4]	 fd_sjw_h[3:0]	 fd_sjw = {fd_sjw_h[3:0], fd_sjw_l[1:0]} 
	 	[3:0]	 sjw_h[3:0]  sjw = {sjw_h[3:0], sjw_l[1:0]}  

	 */

	r |= f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x19,
					      GENMASK(3, 0), sjw >> 2);
#else
	r |= f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x19,
					      GENMASK(3, 0), 0);
#endif

	r |= f81605_set_sja1000_register(priv->dev, dev->dev_id, 0x05, btr2);
	r |= f81605_set_sja1000_register(priv->dev, dev->dev_id, 0x09, btr3);

	// can2.0 delay ack
	r |= f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x77,
					      BIT(4), BIT(4));

	if (r) {
		netdev_info(
			dev,
			"setting id:%d BTR0=0x%02x BTR1=0x%02x failed: %d\n",
			dev->dev_id, btr0, btr1, r);
	}

	return r;
}

static int f81605_set_data_bittiming(struct net_device *dev)
{
	struct f81605_port_priv *priv = netdev_priv(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0)
	const struct can_bittiming *dbt = &priv->can.fd.data_bittiming;
#else
	const struct can_bittiming *dbt = &priv->can.data_bittiming;
#endif
	u32 btr5, btr4, btr7, seg1, seg2, sjw, brp;
	u8 tmp;

	seg1 = dbt->prop_seg + dbt->phase_seg1 - 1;
	seg2 = dbt->phase_seg2 - 1;
	brp = dbt->brp - 1;
	sjw = dbt->sjw - 1;

	if (force_sjw_max)
		sjw = min(seg2, f81605_data_bittiming_const.sjw_max);

	btr5 = (brp & 0x3f) | ((sjw & 0x3) << 6);
	btr7 = (seg1 & 0xf) | ((seg2 & 0x7) << 4);

	if (disable_ssp >= 1) {
		f81605_set_mask_sja1000_register(
			priv->dev, dev->dev_id, 0x77,
			BIT(5) | BIT(2) | BIT(1) | BIT(0), 0);
	} else {
		if (disable_ssp <= -1) {
			f81605_set_mask_sja1000_register(
				priv->dev, dev->dev_id, 0x77,
				BIT(5) | BIT(2) | BIT(1) | BIT(0),
				BIT(5) | BIT(2) | BIT(1) | BIT(0));

			// auto offset
#if 1
			f81605_set_mask_sja1000_register(priv->dev,
							 dev->dev_id, 0x75,
							 GENMASK(5, 4),
							 auto_ssp_offset << 4);

#else
			f81605_set_sja1000_register(priv->dev, dev->dev_id,
						    0x75,
						    auto_ssp_offset << 4);
#endif
		} else if (disable_ssp == 0) {
			f81605_set_mask_sja1000_register(
				priv->dev, dev->dev_id, 0x77,
				BIT(5) | BIT(2) | BIT(1) | BIT(0),
				BIT(5) | BIT(2) | BIT(0));

			if (ssp_value < 2 && ssp_value != -1) {
				netdev_warn(
					dev,
					"%s: custom ssp_value err (%x), must larger than 2\n",
					__func__, ssp_value);
				f81605_set_sja1000_register(
					priv->dev, dev->dev_id, 0x78, 9);
			} else {
				if (ssp_value == -1) {
					f81605_set_sja1000_register(
						priv->dev, dev->dev_id, 0x78,
						9);

					netdev_info(dev,
						    "%s: manual ssp: %d\n",
						    __func__, 9);
				} else {
					f81605_set_sja1000_register(
						priv->dev, dev->dev_id, 0x78,
						ssp_value);

					netdev_info(dev,
						    "%s: manual ssp: %d\n",
						    __func__, ssp_value);
				}
			}
		}
	}

	netdev_info(dev, "%s: fd seg1: %x, seg2: %x, brp: %x\n", __func__,
		    seg1, seg2, brp);

	/*
		Index 0x18 (BTR5 : Bus Timing Register 5)			
		Bit Name	Description default
		[7:6]	fd_sjw_l[1:0]	fd_sjw = {fd_sjw_h[3:0], fd_sjw_l[1:0]} 
		[5:0]	fd_brp		

		Index 0x1A (BTR7 : Bus Timing Register 7)		
		Bit	Name	Description
		[7]	Reserved	
		[6:4]	fd_tseg_2[2:0]	fd_tseg_2 = {fd_tseg_2[6:3], fd_tseg2[2:0]}
		[3:0]	fd_tseg_1[3:0]	fd_tseg_1 = {fd_tseg_1[7:4], fd_tseg_1[3:0]}
	*/

	f81605_set_sja1000_register(priv->dev, dev->dev_id, 0x18, btr5);
	f81605_set_sja1000_register(priv->dev, dev->dev_id, 0x1a, btr7);

	btr4 = 0;

#if F81605_EXTEND_BTR
	/*
	 	Index 0x19 (BTR6 : Bus Timing Register 6)			 
	 	Bit Name	 Description default
	 	[7:4]	 fd_sjw_h[3:0]	 fd_sjw = {fd_sjw_h[3:0], fd_sjw_l[1:0]} 
	 	[3:0]	 sjw_h[3:0]  sjw = {sjw_h[3:0], sjw_l[1:0]}  

	 */

	f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x19,
					 GENMASK(7, 4), sjw << 2);

	/*
		Index 0x0A (BTR4: Bus Timing Register4 )		
		Bit Name	Description
		[7:4]	fd_tseg_2[6:3]	fd_tseg_2 = {fd_tseg_2[6:3], fd_tseg_2[2:0]}
		[3:0]	fd_tseg_1[7:4]	fd_tseg_1 = {fd_tseg_1[7:4], fd_tseg_1[3:0]}
	 */

	btr4 |= ((GENMASK(3, 0) & (seg1 >> 4))) |
		(GENMASK(7, 4) & (seg2 << 1));

#else
	f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x19,
					 GENMASK(7, 4), 0);

#endif

	f81605_set_sja1000_register(priv->dev, dev->dev_id, 0x0a, btr4);

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO) {
		f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x77,
						 BIT(3), 0);
	} else {
		f81605_set_mask_sja1000_register(priv->dev, dev->dev_id, 0x77,
						 BIT(3), BIT(3));
	}

	netdev_info(dev, "FD DATA: %d\n", dbt->bitrate);
	netdev_info(dev, "setting DATA BTR5=%02xh BTR7=%02xh, BTR4=%02xh\n",
		    btr5, btr7, btr4);

	f81605_get_sja1000_register(priv->dev, dev->dev_id, 0x0a, &tmp);
	netdev_info(dev, "%s: (BTR4)0x0ah: %02xh\n", __func__, tmp);

	f81605_get_sja1000_register(priv->dev, dev->dev_id, 0x18, &tmp);
	netdev_info(dev, "%s: (BTR5)0x18h: %02xh\n", __func__, tmp);

	f81605_get_sja1000_register(priv->dev, dev->dev_id, 0x1a, &tmp);
	netdev_info(dev, "%s: (BTR7)0x1ah: %02xh\n", __func__, tmp);

	return 0;
}

static int f81605_set_mode(struct net_device *netdev, enum can_mode mode)
{
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		err = f81605_start(netdev);
		if (!err && netif_queue_stopped(netdev))
			netif_wake_queue(netdev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return err;
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

#if CAN_STS_POLL
static void f81605_oneshot_err_handler(struct net_device *netdev, u8 status,
				       u8 ecc, u8 *tx_err)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	u8 old_mod, *tec, is_called;
	enum can_state state;
	unsigned long flags;
	int r = 0;

	if (!(priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT))
		return;

	if (state >= CAN_STATE_BUS_OFF)
		return;

	if (status & SR_TS)
		return;

	spin_lock_irqsave(&priv->lock, flags);
	is_called = priv->is_tx_callbacked;
	spin_unlock_irqrestore(&priv->lock, flags);

	if (!is_called)
		return;

	cancel_delayed_work_sync(&priv->delayed_oneshot_work);

	//if (ecc != CAN_ERR_PROT_LOC_ACK)
	//	goto do_send;

	tec = &priv->old_oneshot_tec;

	r |= f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					 SJA1000_MOD, &old_mod);

	if (r)
		goto do_send;

	if (old_mod & MOD_RM)
		goto do_send;

	if (*tec >= 128)
		goto do_send;
	else if (*tec + 8 >= 128)
		*tec = 128;
	else
		*tec += 8;

	if (tx_err)
		*tx_err = *tec;

	f81605_set_sja1000_register(priv->dev, netdev->dev_id, SJA1000_MOD,
				    MOD_RM);

	f81605_set_sja1000_register(priv->dev, netdev->dev_id, SJA1000_TXERR,
				    *tec);

	f81605_set_sja1000_register(priv->dev, netdev->dev_id, SJA1000_MOD,
				    old_mod);

do_send:
	spin_lock_irqsave(&priv->lock, flags);

	if (priv->is_tx_callbacked) {
		priv->is_tx_callbacked = false;

		netif_wake_queue(netdev);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void f81605_rx_sts_poll(struct urb *urb)
{
	struct net_device *dev = urb->context;
	struct f81605_port_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf, f = { 0 };
	struct sk_buff *skb;
	enum can_state state = priv->can.state;
	enum can_state rx_state, tx_state;
	const u8 max_rx_err = 128;
	const u8 ewl = 96;
	unsigned int rxerr, txerr;
	u8 ecc, isrc, status, tx_err, rx_err, max_err, tmp, alc;
	bool is_err = false;
	u8 *data;

	data = urb->transfer_buffer;

#if 0
	//if (dev->dev_id == 1) {
		netdev_info(dev, "%s: rx, %d\n", __func__, urb->actual_length);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, data,
			       urb->actual_length, true);
	//}
#endif

	cf = &f;
	tx_err = cf->data[6] = data[7];
	rx_err = cf->data[7] = data[6];
	//ewl = data[8];
	rx_err = min_t(u8, rx_err, max_rx_err);
	max_err = max_t(u8, tx_err, rx_err);

	isrc = data[2];
	status = data[1];

	if (isrc & IRQ_DOI) {
		/* data overrun interrupt */
		netdev_dbg(dev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;

		/* auto cleared 108/109h */
	}

	if (isrc & IRQ_EI) {
		if (status & SR_BS)
			state = CAN_STATE_BUS_OFF;
		else
			state = CAN_STATE_ERROR_ACTIVE;
	}

	if (isrc & IRQ_EPI) {
		/* error passive interrupt */
		netdev_dbg(dev, "error passive interrupt\n");

		if ((priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) &&
		    tx_err >= 128)
			state = CAN_STATE_ERROR_PASSIVE;
		else if (state == CAN_STATE_ERROR_PASSIVE)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_PASSIVE;
	}

	if (state < CAN_STATE_ERROR_PASSIVE) {
		if (max_err >= ewl)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_ACTIVE;
	}

	if (isrc & IRQ_BEI) {
		f81605_get_sja1000_register(priv->dev, dev->dev_id,
					    SJA1000_ECC, &ecc);

		if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) {
			/* bus error interrupt */
			priv->can.can_stats.bus_error++;
			stats->rx_errors++;

			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

			/* set error type */
			switch (ecc & ECC_MASK) {
			case ECC_BIT:
				cf->data[2] |= CAN_ERR_PROT_BIT;
				break;
			case ECC_FORM:
				cf->data[2] |= CAN_ERR_PROT_FORM;
				break;
			case ECC_STUFF:
				cf->data[2] |= CAN_ERR_PROT_STUFF;
				break;
			default:
				break;
			}

			/* set error location */
			cf->data[3] = ecc & ECC_SEG;

			/* Error occurred during transmission? */
			if ((ecc & ECC_DIR) == 0)
				cf->data[2] |= CAN_ERR_PROT_TX;
		}

		if ((ecc & ECC_DIR) == 0) {
			f81605_oneshot_err_handler(dev, status, ecc & ECC_SEG,
						   &tx_err);

			cf->data[6] = tx_err;
		}
	}

	if (isrc & IRQ_ALI) {
		f81605_get_sja1000_register(priv->dev, dev->dev_id,
					    SJA1000_ALC, &alc);

		/* arbitration lost interrupt */
		netdev_dbg(dev, "arbitration lost interrupt\n");

		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = alc & 0x1f;
	}

	if (state != priv->can.state) {
		tx_state = txerr >= rxerr ? state : 0;
		rx_state = txerr <= rxerr ? state : 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		can_change_state(dev, cf, tx_state, rx_state);
#else
		if (state == CAN_STATE_ERROR_WARNING) {
			//priv->can.can_stats.error_warning++;
			stats->error_warning++;
			cf->data[1] = (txerr > rxerr) ?
					      CAN_ERR_CRTL_TX_WARNING :
					      CAN_ERR_CRTL_RX_WARNING;
		} else {
			//priv->can.can_stats.error_passive++;
			stats->error_passive++;
			cf->data[1] = (txerr > rxerr) ?
					      CAN_ERR_CRTL_TX_PASSIVE :
					      CAN_ERR_CRTL_RX_PASSIVE;
		}
#endif

		if (state == CAN_STATE_BUS_OFF)
			can_bus_off(dev);

		is_err = true;
	}

	if (is_err || cf->can_id) {
		skb = alloc_can_err_skb(dev, &cf);
		if (skb == NULL) {
			netdev_err(dev, "%s: no mem to report error packet\n",
				   __func__);
			return;
		}

		f.can_id |= CAN_ERR_FLAG;
		f.can_dlc = CAN_ERR_DLC;
		memcpy(cf, &f, sizeof(*cf));

		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}

	if (data[6] > max_rx_err) {
		netdev_dbg(dev, "%s: cur rx_err: %d %d\n", __func__, data[6],
			   rx_err);

		f81605_get_sja1000_register(priv->dev, dev->dev_id,
					    SJA1000_MOD, &tmp);

		f81605_set_sja1000_register(priv->dev, dev->dev_id,
					    SJA1000_MOD, MOD_RM);

		f81605_set_sja1000_register(priv->dev, dev->dev_id,
					    SJA1000_RXERR, max_rx_err);

		f81605_set_sja1000_register(priv->dev, dev->dev_id,
					    SJA1000_MOD, tmp);
	}
}
#else
static void f81605_rx_sts_callback(struct urb *urb)
{
	struct net_device *dev = urb->context;
	struct f81605_port_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state state = priv->can.state;
	enum can_state rx_state, tx_state;
	unsigned int rxerr, txerr;
	uint8_t ecc, alc, isrc, status;
	unsigned long flags;
	u8 *data;

	data = urb->transfer_buffer;

#if 0
	netdev_info(dev, "%s: rx\n", __func__);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, data,
		       urb->actual_length, true);
#endif

	skb = alloc_can_err_skb(dev, &cf);
	if (skb == NULL) {
		netdev_err(dev, "%s: no mem to report error packet\n",
			   __func__);
		return;
	}

	cf->data[6] = data[7];
	cf->data[7] = data[6];

	isrc = data[2];
	status = data[1];

	if (isrc & IRQ_DOI) {
		/* data overrun interrupt */
		netdev_dbg(dev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;

		/* auto cleared 108/109h */
	}

	if (isrc & IRQ_EI) {
		/* error warning interrupt */
		netdev_dbg(dev, "error warning interrupt\n");

		if (status & SR_BS)
			state = CAN_STATE_BUS_OFF;
		else if (status & SR_ES)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_ACTIVE;
	}

	if (isrc & IRQ_BEI) {
		/* bus error interrupt */
		priv->can.can_stats.bus_error++;
		stats->rx_errors++;

		spin_lock_irqsave(&priv->lock, flags);
		priv->ecc = ecc = data[5];
		spin_unlock_irqrestore(&priv->lock, flags);

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		/* set error type */
		switch (ecc & ECC_MASK) {
		case ECC_BIT:
			cf->data[2] |= CAN_ERR_PROT_BIT;
			break;
		case ECC_FORM:
			cf->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case ECC_STUFF:
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		default:
			break;
		}

		/* set error location */
		cf->data[3] = ecc & ECC_SEG;

		/* Error occurred during transmission? */
		if ((ecc & ECC_DIR) == 0)
			cf->data[2] |= CAN_ERR_PROT_TX;
	}

	if (isrc & IRQ_EPI) {
		/* error passive interrupt */
		netdev_dbg(dev, "error passive interrupt\n");

		if (state == CAN_STATE_ERROR_PASSIVE)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_PASSIVE;
	}

	if (isrc & IRQ_ALI) {
		/* arbitration lost interrupt */
		netdev_dbg(dev, "arbitration lost interrupt\n");

		spin_lock_irqsave(&priv->lock, flags);
		priv->alc = alc = data[4];
		spin_unlock_irqrestore(&priv->lock, flags);

		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = alc & 0x1f;
	}

#if 0
	static enum can_state old_state;

	if (old_state != state && !dev->dev_id) {
		netdev_info(dev, "%s: state change %d %d\n", __func__,
			    old_state, state);
		old_state = state;
	}
#endif

	if (state != priv->can.state) {
		tx_state = txerr >= rxerr ? state : 0;
		rx_state = txerr <= rxerr ? state : 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		can_change_state(dev, cf, tx_state, rx_state);
#else
		if (state == CAN_STATE_ERROR_WARNING) {
			//priv->can.can_stats.error_warning++;
			stats->error_warning++;
			cf->data[1] = (txerr > rxerr) ?
					      CAN_ERR_CRTL_TX_WARNING :
					      CAN_ERR_CRTL_RX_WARNING;
		} else {
			//priv->can.can_stats.error_passive++;
			stats->error_passive++;
			cf->data[1] = (txerr > rxerr) ?
					      CAN_ERR_CRTL_TX_PASSIVE :
					      CAN_ERR_CRTL_RX_PASSIVE;
		}
#endif

		if (state == CAN_STATE_BUS_OFF)
			can_bus_off(dev);
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);
}
#endif

static void f81605_rx_data_frame(struct urb *urb)
{
	struct net_device *dev = urb->context;
	struct net_device_stats *stats = &dev->stats;
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u8 *data, *ptr;
	uint32_t consumed_size = 0;
	uint32_t fi, dreg;
	canid_t id;
	int i;

	ptr = data = urb->transfer_buffer;

	if (urb->actual_length < 3) {
		netdev_err(dev, "%s: rx len(%d) < 3\n", __func__,
			   urb->actual_length);

		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, data,
			       urb->actual_length, true);

		return;
	}

#if CAN_DELAYED_RX_DBG
	netdev_info(dev, "%s: rx: %d\n", __func__, urb->actual_length);

	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, ptr,
		       urb->actual_length, true);
#endif

	while (consumed_size < urb->actual_length) {
		data = &ptr[consumed_size];

		fi = data[SJA1000_FI - SJA1000_FI];
		if (fi & F81605_FI_EDL)
			skb = alloc_canfd_skb(dev, &cf);
		else
			skb = alloc_can_skb(dev, (struct can_frame **)&cf);
		if (skb == NULL) {
			netdev_err(dev, "%s: alloc_can_skb failed\n",
				   __func__);
			return;
		}

		if (fi & SJA1000_FI_FF) {
			/* extended frame format (EFF) */
			dreg = SJA1000_EFF_BUF - SJA1000_FI;
			id = (data[SJA1000_ID1 - SJA1000_FI] << 21) |
			     (data[SJA1000_ID2 - SJA1000_FI] << 13) |
			     (data[SJA1000_ID3 - SJA1000_FI] << 5) |
			     (data[SJA1000_ID4 - SJA1000_FI] >> 3);
			id |= CAN_EFF_FLAG;

			consumed_size += 5;
		} else {
			/* standard frame format (SFF) */
			dreg = SJA1000_SFF_BUF - SJA1000_FI;
			id = (data[SJA1000_ID1 - SJA1000_FI] << 3) |
			     (data[SJA1000_ID2 - SJA1000_FI] >> 5);

			consumed_size += 3;
		}

		// RX ESI check
		if (fi & F81605_FI_EDL) {
			if (fi & F81605_FI_BRS)
				cf->flags |= CANFD_BRS;

			if (fi & SJA1000_FI_FF) {
				if (data[SJA1000_ID4 - SJA1000_FI] & BIT(0))
					cf->flags |= CANFD_ESI;
			} else {
				if (data[SJA1000_ID2 - SJA1000_FI] & BIT(0))
					cf->flags |= CANFD_ESI;
			}
		}

		if (fi & SJA1000_FI_RTR) {
			id |= CAN_RTR_FLAG;
		} else {
			cf->len = can_dlc2len(fi & 0x0f);

			for (i = 0; i < cf->len; i++)
				cf->data[i] = data[dreg++];
		}

		cf->can_id = id;

		stats->rx_packets++;
		stats->rx_bytes += cf->len;
		consumed_size += cf->len;

		//netdev_info(dev, "rx consumed_size: %d, cf->len: %d\n",
		//	    consumed_size, cf->len);

		netif_rx(skb);
	}
}

#if !CAN_STS_POLL
static void f81605_read_sts_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	int r;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		//netdev_dbg(netdev, "%s: URB ok (%d) %d\n", __func__,
		//	   urb->status, urb->actual_length);
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

	f81605_rx_sts_callback(urb);

resubmit_urb:
	r = usb_submit_urb(urb, GFP_ATOMIC);
	if (r == -ENODEV) {
		netif_device_detach(netdev);
	} else if (r) {
		netdev_err(netdev, "failed resubmitt sts bulk urb: %d\n", r);
	}
}
#endif

/* Callback for reading data from device
 *
 * Check urb status, call read function and resubmit urb read operation.
 */
static void f81605_read_rx_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	int r;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		//netdev_dbg(netdev, "%s: URB ok (%d) %d\n", __func__,
		//	   urb->status, urb->actual_length);
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

	f81605_rx_data_frame(urb);

resubmit_urb:

#if CAN_DELAYED_RX_DBG
	struct f81605_port_priv *priv = netdev_priv(netdev);

	schedule_delayed_work(&priv->delayed_rx_work, msecs_to_jiffies(5000));
#else
	r = usb_submit_urb(urb, GFP_ATOMIC);
	if (r == -ENODEV) {
		netif_device_detach(netdev);
	} else if (r) {
		netdev_err(netdev, "failed resubmit read bulk urb: %d\n", r);
	}
#endif
}

/* Callback handler for write operations
 *
 * Free allocated buffers, check transmit status and
 * calculate statistic.
 */
static void f81605_write_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	struct f81605_port_priv *priv = netdev_priv(netdev);
	unsigned long flags;
	int r, i;
	u8 *data;

	if (!netif_device_present(netdev))
		return;

	for (i = 0; i < F81605_MAX_TX_URBS; ++i) {
		if (priv->write_urb[i] == urb)
			break;
	}

	if (unlikely(i >= F81605_MAX_TX_URBS)) {
		netdev_err(netdev, "%s: i(%d) >= F81605_MAX_TX_URBS\n",
			   __func__, i);
		return;
	}

	data = urb->transfer_buffer;
	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += can_dlc2len(data[1]);

	switch (urb->status) {
	case 0: /* success */
		if (debug) {
			netdev_info(netdev, "%s: URB ok (%d)(%d)\n", __func__,
				    urb->status, i);
		}
		break;

	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		if (debug) {
			netdev_info(netdev, "%s: URB aborted (%d)(%d)\n",
				    __func__, urb->status, i);
		}

		spin_lock_irqsave(&priv->lock, flags);
		set_bit(i, &priv->write_urb_sts);
		spin_unlock_irqrestore(&priv->lock, flags);

		return;

	default:
		if (debug) {
			netdev_info(netdev, "Tx URB aborted (%d)(%d)\n",
				    urb->status, i);
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	r = can_get_echo_skb(netdev, i, NULL);
#else
	r = can_get_echo_skb(netdev, i);
#endif
#else
	can_get_echo_skb(netdev, i);
#endif

	spin_lock_irqsave(&priv->lock, flags);

	priv->is_tx_callbacked = true;

	if (!(priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)) {
		if (!priv->write_urb_sts)
			netif_wake_queue(netdev);
	} else {
		schedule_delayed_work(&priv->delayed_oneshot_work,
				      msecs_to_jiffies(300));
	}

	set_bit(i, &priv->write_urb_sts);

	spin_unlock_irqrestore(&priv->lock, flags);
}

#if CAN_TIMER_DBG
static void f81605_delayed_timer_work(struct work_struct *work)
{
	struct f81605_port_priv *priv;
	struct net_device *dev;
	enum can_state state;
	u8 tmp;
	int r;

	priv = container_of(work, struct f81605_port_priv,
			    delayed_timer_work.work);
	dev = priv->netdev;
	state = priv->can.state;

	r = f81605_get_sja1000_register(priv->dev, dev->dev_id, 0x76, &tmp);

	if (tmp != priv->auto_ssp) {
		priv->auto_ssp = tmp;
		netdev_info(dev, "%s: auto ssp: %02xh\n", __func__, tmp);
	}

	if (state < CAN_STATE_BUS_OFF) {
		schedule_delayed_work(&priv->delayed_timer_work,
				      msecs_to_jiffies(1000));
	}
}
#endif

#if CAN_STS_POLL
static void f81605_delayed_sts_work(struct work_struct *work)
{
	static const u8 reg_tbl[] = { 0xff,	     SJA1000_SR,    SJA1000_IR,
				      0xff,	     0xff,	    0xff,
				      SJA1000_RXERR, SJA1000_TXERR, 0xff };
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	struct urb sts_urb;
	u8 sts_data[16];
	int r, i;

	priv = container_of(work, struct f81605_port_priv,
			    delayed_sts_work.work);
	netdev = priv->netdev;

	memset(&sts_urb, 0, sizeof(sts_urb));
	sts_urb.context = netdev;
	sts_urb.transfer_buffer = sts_data;
	sts_urb.actual_length = sizeof(reg_tbl);

	for (i = 0; i < sizeof(reg_tbl); ++i) {
		if (reg_tbl[i] == 0xff)
			continue;

		r = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
						reg_tbl[i], &sts_data[i]);
		if (r) {
			netdev_dbg(netdev, "%s: read reg: %xh failed: %d\n",
				   __func__, reg_tbl[i], r);
			goto end;
		}
	}

	f81605_rx_sts_poll(&sts_urb);

	if (sts_data[1] & SR_DOS) {
		// sr, check is overrun
		f81605_set_sja1000_register(priv->dev, netdev->dev_id,
					    SJA1000_CMR, CMD_CDO);
	}

end:
	if (priv->can.state < CAN_STATE_BUS_OFF) {
		queue_delayed_work(priv->wq, &priv->delayed_sts_work,
				   CAN_STS_POLL_TIMER);
	}
}
#endif

#if CAN_DELAYED_RX_DBG
static void f81605_delayed_rx_work(struct work_struct *work)
{
	struct f81605_port_priv *priv;
	struct net_device *dev;
	u8 tmp;
	int r;

	priv = container_of(work, struct f81605_port_priv,
			    delayed_rx_work.work);
	dev = priv->netdev;

	//netdev_info(dev, "%s: reg\n", __func__);

	if (dev->dev_id == 1) {
		r = f81605_get_sja1000_register(priv->dev, dev->dev_id,
						SJA1000_RMC, &tmp);

		netdev_info(dev, "%s: rmc: %d\n", __func__, tmp);
	}

	r = usb_submit_urb(priv->read_rx_urb[0], GFP_ATOMIC);
	if (r == -ENODEV) {
		netif_device_detach(dev);
	} else if (r) {
		netdev_err(dev, "failed resubmit delayed read bulk urb: %d\n",
			   r);
	}
}
#endif

static void f81605_oneshot_work(struct work_struct *work)
{
	struct f81605_port_priv *priv;
	enum can_state state;
	unsigned long flags;
	u8 tx_err;

	priv = container_of(work, struct f81605_port_priv,
			    delayed_oneshot_work.work);
	state = priv->can.state;

	if (state >= CAN_STATE_BUS_OFF)
		return;

	f81605_get_sja1000_register(priv->dev, priv->netdev->dev_id,
				    SJA1000_TXERR, &tx_err);

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->is_tx_callbacked) {
		priv->is_tx_callbacked = false;
		priv->old_oneshot_tec = tx_err;

		netif_wake_queue(priv->netdev);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
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

/* Send data to device */
static netdev_tx_t f81605_start_xmit(struct sk_buff *skb,
				     struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	struct net_device_stats *stats = &netdev->stats;
	u8 *ptr, fi;
	u32 id;
	int status, idx, tx_size;
	unsigned long flags;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;
#endif

	spin_lock_irqsave(&priv->lock, flags);

	idx = (int)find_first_bit(&priv->write_urb_sts, F81605_MAX_TX_URBS);
	if (idx >= F81605_MAX_TX_URBS) {
		netdev_err(netdev, "%s: no enough space: %d\n", __func__, idx);
		return NETDEV_TX_OK;
	}

	clear_bit(idx, &priv->write_urb_sts);

	if (!priv->write_urb_sts ||
	    (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT))
		netif_stop_queue(netdev);

	spin_unlock_irqrestore(&priv->lock, flags);

	ptr = priv->bulk_write_buffer[idx];

	if (can_is_canfd_skb(skb)) {
		fi = can_len2dlc(cf->len);
		tx_size = can_dlc2len(fi);

		fi |= F81605_FI_EDL;

		if (cf->flags & CANFD_BRS)
			fi |= F81605_FI_BRS;

	} else {
		if (cf->can_id & CAN_RTR_FLAG) {
			tx_size = 0;
			fi = SJA1000_FI_RTR;
		} else {
			tx_size = fi = cf->len;
		}
	}

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		ptr[0] = CMD_SRR;
	else
		ptr[0] = CMD_TR;

	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		ptr[0] |= CMD_AT;

	ptr[1] = fi;

	if (cf->can_id & CAN_EFF_FLAG) {
		id = (cf->can_id & CAN_ERR_MASK) << 3;
		ptr[1] |= SJA1000_FI_FF;
		ptr[4] = (id >> 24) & 0xff;
		ptr[5] = (id >> 16) & 0xff;
		ptr[6] = (id >> 8) & 0xff;
		ptr[7] = (id >> 0) & 0xff;
	} else {
		id = (cf->can_id & CAN_ERR_MASK) << 5;
		ptr[4] = (id >> 8) & 0xff;
		ptr[5] = (id >> 0) & 0xff;
	}

	memcpy(&ptr[8], cf->data, tx_size);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	can_put_echo_skb(skb, netdev, idx, 0);
#else
	can_put_echo_skb(skb, netdev, idx);
#endif

	priv->write_urb[idx]->transfer_buffer_length = tx_size + 8;

#if 0
	netdev_info(netdev, "%s: tx: %d\n", __func__,
		    priv->write_urb[idx]->transfer_buffer_length);

	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, ptr,
		       priv->write_urb[idx]->transfer_buffer_length, true);
#endif

	status = usb_submit_urb(priv->write_urb[idx], GFP_ATOMIC);
	if (status) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
		can_free_echo_skb(netdev, idx, NULL);
#else
		can_free_echo_skb(netdev, idx);
#endif
		dev_kfree_skb(skb);

		spin_lock_irqsave(&priv->lock, flags);
		set_bit(idx, &priv->write_urb_sts);
		spin_unlock_irqrestore(&priv->lock, flags);

		netdev_err(netdev, "%s: failed resubmit tx urb: %d\n",
			   __func__, status);

		stats->tx_dropped++;

		if (status == -ENODEV)
			netif_device_detach(netdev);
	}

	return NETDEV_TX_OK;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
static int f81605_get_berr_counter(const struct net_device *netdev,
				   struct can_berr_counter *bec)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int status;
	u8 txerr;
	u8 rxerr;

	if (!(priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)) {
		status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
						     SJA1000_TXERR, &txerr);
		if (status)
			return status;
	} else {
		txerr = priv->old_oneshot_tec;
	}

	status = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					     SJA1000_RXERR, &rxerr);
	if (status)
		return status;

	bec->txerr = txerr;
	bec->rxerr = rxerr;

	return 0;
}
#endif

static int f81605_prepare_urbs(struct net_device *netdev)
{
#if !CAN_STS_POLL
	static u8 bulk_sts_in_addr[F81605_MAX_DEV] = { 0x81, 0x83 };
#endif
	static u8 bulk_rx_in_addr[F81605_MAX_DEV] = { 0x82, 0x84 };
	static u8 bulk_out_addr[F81605_MAX_DEV] = { 0x01, 0x03 };
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int id = netdev->dev_id;
	int status;
	int i;

	for (i = 0; i < F81605_MAX_RX_URBS; ++i) {
		priv->read_rx_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!priv->read_rx_urb[i]) {
			status = -ENOMEM;
			goto error;
		}

		usb_fill_bulk_urb(
			priv->read_rx_urb[i], priv->dev,
			usb_rcvbulkpipe(priv->dev, bulk_rx_in_addr[id]),
			priv->bulk_read_rx_buffer[i], F81605_RX_BULK_SIZE,
			f81605_read_rx_bulk_callback, netdev);

#if !CAN_STS_POLL

		priv->read_sts_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!priv->read_sts_urb[i]) {
			status = -ENOMEM;
			goto error;
		}

		usb_fill_bulk_urb(
			priv->read_sts_urb[i], priv->dev,
			usb_rcvbulkpipe(priv->dev, bulk_sts_in_addr[id]),
			priv->bulk_read_sts_buffer[i], F81605_RX_BULK_SIZE,
			f81605_read_sts_bulk_callback, netdev);
#endif
	}

	priv->write_urb_sts = 0;
	for (i = 0; i < F81605_MAX_TX_URBS; ++i) {
		set_bit(i, &priv->write_urb_sts);

		priv->write_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!priv->write_urb[i]) {
			status = -ENOMEM;
			goto error;
		}

		usb_fill_bulk_urb(
			priv->write_urb[i], priv->dev,
			usb_sndbulkpipe(priv->dev, bulk_out_addr[id]),
			priv->bulk_write_buffer[i], F81605_TX_FRAME_SIZE,
			f81605_write_bulk_callback, netdev);
	}

	return 0;

error:
	/* todo error */
	return status;
}

static void f81605_remove_urbs(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int i;

	for (i = 0; i < F81605_MAX_TX_URBS; ++i)
		usb_free_urb(priv->write_urb[i]);

	for (i = 0; i < F81605_MAX_RX_URBS; ++i) {
		usb_free_urb(priv->read_rx_urb[i]);
#if !CAN_STS_POLL
		usb_free_urb(priv->read_sts_urb[i]);
#endif
	}
}

static int f81605_register_urbs(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int status;
	int i;

	for (i = 0; i < F81605_MAX_RX_URBS; ++i) {
#if CAN_DELAYED_RX_DBG
		schedule_delayed_work(&priv->delayed_rx_work,
				      msecs_to_jiffies(5000));
#else
		status = usb_submit_urb(priv->read_rx_urb[i], GFP_KERNEL);
		if (status) {
			pr_err("%s: submit read data urb failed: %d\n",
			       __func__, status);
			return status;
		}
#endif

#if !CAN_STS_POLL
		status = usb_submit_urb(priv->read_sts_urb[i], GFP_KERNEL);
		if (status) {
			pr_err("%s: submit read sts urb failed: %d\n",
			       __func__, status);
			return status;
		}
#endif
	}

	return 0;
}

static void f81605_unregister_urbs(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);
	int i;

	for (i = 0; i < F81605_MAX_TX_URBS; ++i)
		usb_kill_urb(priv->write_urb[i]);

	for (i = 0; i < F81605_MAX_RX_URBS; ++i) {
		usb_kill_urb(priv->read_rx_urb[i]);
#if !CAN_STS_POLL
		usb_kill_urb(priv->read_sts_urb[i]);
#endif
	}
}

static ssize_t ssp_val_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	u8 ssp_set, manual_ssp, auto_ssp, auto_ssp_offset_v;
	int r;

	netdev = to_net_dev(dev);
	priv = netdev_priv(netdev);

	r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, 0x77,
					&ssp_set);
	if (r)
		return r;

	r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, 0x78,
					&manual_ssp);
	if (r)
		return r;

	r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, 0x76,
					&auto_ssp);
	if (r)
		return r;

	r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, 0x75,
					&auto_ssp_offset_v);
	if (r)
		return r;

	auto_ssp_offset_v = (auto_ssp_offset_v >> 4) & 0x03;

	if (disable_ssp > 0) {
		return sprintf(buf, "%d: ssp disable, ssp_set: %xh\n",
			       netdev->dev_id, ssp_set);
	}

	if (disable_ssp <= -1) {
		return sprintf(
			buf,
			"%d(AUTO): auto ssp:%lxh, ssp_set: %xh, manu: %lxh, offset: %xh\n",
			netdev->dev_id, auto_ssp & GENMASK(6, 0), ssp_set,
			manual_ssp & GENMASK(6, 0), auto_ssp_offset_v);
	}

	return sprintf(
		buf,
		"%d(MANUAL): auto ssp:%lxh, ssp_set: %xh, manu: %lxh offset: %xh\n",
		netdev->dev_id, auto_ssp & GENMASK(6, 0), ssp_set,
		manual_ssp & GENMASK(6, 0), auto_ssp_offset_v);
}

static ssize_t bus_loading_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	const int timeout_ms = 500;
	ktime_t time_start;
	int r, loading;
	s64 time_diff;
	u8 load[2];

	netdev = to_net_dev(dev);
	priv = netdev_priv(netdev);

	// release lock
	r = f81605_set_sja1000_register(priv->dev, netdev->dev_id, 0x7a,
					BIT(7));
	if (r)
		return r;

	time_start = ktime_get();

	while ((time_diff = ktime_to_ms(ktime_sub(ktime_get(), time_start))) <
	       timeout_ms) {
		r = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
						0x7a, &load[0]);
		if (r)
			return r;

		if (load[0] & BIT(7)) // locked
			break;
	}

	if (time_diff >= timeout_ms) {
		pr_info("%s: locked failed, time_diff: %lld\n", __func__,
			time_diff);

		return -EBUSY;
	}

	r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, 0x7a,
					&load[0]);
	if (r)
		return r;

	r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, 0x7b,
					&load[1]);
	if (r)
		return r;

	load[0] &= ~BIT(7);
	loading = (load[0] << 8) | load[1];
	loading = (loading * 100) / 32767;

	return sprintf(buf, "%d%%\n", loading);
}

static ssize_t reg_dump_all_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	int i, r, off = 0;
	u8 tmp;

	netdev = to_net_dev(dev);
	priv = netdev_priv(netdev);

	for (i = 0; i < 17; ++i) {
		if (i == 0)
			off += sprintf(&buf[off], "   ");
		else
			off += sprintf(&buf[off], "%02X ", i - 1);
	}

	off += sprintf(&buf[off], "\n");

	for (i = 0; i < 0x80; ++i) {
		r = f81605_get_sja1000_register(priv->dev, netdev->dev_id, i,
						&tmp);
		if (r)
			return r;

		if ((i % 16) == 0) {
			if (i != 0)
				off += sprintf(&buf[off], "\n");

			off += sprintf(&buf[off], "%02X ", i & 0xf0);
		}

		off += sprintf(&buf[off], "%02X ", tmp);
	}

	off += sprintf(&buf[off], "\n");

	return off;
}

static ssize_t reg_dump_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	uint32_t param[2] = { -1, -1 };
	int r, i;
	u8 *p, tmp;

	for (i = 0; i < 2; ++i) {
		p = strsep((char **)&buf, " ");
		if (!p)
			break;

		r = kstrtoint(p, 0, &param[i]);
		if (r)
			break;
	}

	if (i < 1)
		return -EINVAL;

	if (param[0] < 0 || param[0] >= 0x80)
		return -EINVAL;

	if (i >= 2 && ((param[1] < 0 || param[1] > 0xff)))
		return -EINVAL;

	netdev = to_net_dev(dev);
	priv = netdev_priv(netdev);

	if (i == 1) {
		// read
		r = f81605_get_sja1000_register(priv->dev, netdev->dev_id,
						(u16)param[0], &tmp);
		if (r)
			return r;

		pr_info("%s: read reg: %xh, data: %xh\n", __func__,
			(u16)param[0], tmp);
	} else {
		// write

		r = f81605_set_sja1000_register(priv->dev, netdev->dev_id,
						(u16)param[0], (u8)param[1]);
		if (r)
			return r;

		pr_info("%s: write reg: %xh, data: %xh\n", __func__,
			(u16)param[0], (u8)param[1]);
	}

	return count;
}

static DEVICE_ATTR_RO(ssp_val);
static DEVICE_ATTR_RO(bus_loading);
static DEVICE_ATTR_RO(reg_dump_all);
static DEVICE_ATTR_WO(reg_dump);

/* Open USB device */
static int f81605_open(struct net_device *netdev)
{
	//struct f81605_port_priv *priv = netdev_priv(netdev);
	//unsigned long flags;
	int err;

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
	can_led_event(netdev, CAN_LED_EVENT_OPEN);
#endif
	/* finally start device */
	err = f81605_start(netdev);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(netdev);

		netdev_warn(netdev, "couldn't start device: %d\n", err);

		close_candev(netdev);

		return err;
	}

	netif_start_queue(netdev);

	return 0;
}

/* Close USB device */
static int f81605_close(struct net_device *netdev)
{
	struct f81605_port_priv *priv = netdev_priv(netdev);

	priv->can.state = CAN_STATE_STOPPED;
	f81605_set_reset_mode(netdev);

#if CAN_TIMER_DBG
	cancel_delayed_work_sync(&priv->delayed_timer_work);
#endif

#if CAN_DELAYED_RX_DBG
	cancel_delayed_work_sync(&priv->delayed_rx_work);
#endif

#if CAN_STS_POLL
	cancel_delayed_work_sync(&priv->delayed_sts_work);
#endif

	cancel_delayed_work_sync(&priv->delayed_oneshot_work);

	netif_stop_queue(netdev);
	f81605_unregister_urbs(netdev);
	close_candev(netdev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
	can_led_event(netdev, CAN_LED_EVENT_STOP);
#endif

	return 0;
}

static int f81605_ioctl_write(struct net_device *netdev, struct ifreq *ifr)
{
	struct f81605_reg_ctrl reg_ctrl;
	struct f81605_port_priv *priv;

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

	return f81605_set_register(priv->dev, reg_ctrl.reg, reg_ctrl.data);
}

static int f81605_ioctl_read(struct net_device *netdev, struct ifreq *ifr)
{
	struct f81605_reg_ctrl reg_ctrl;
	struct f81605_port_priv *priv;
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

	r = f81605_get_register(priv->dev, reg_ctrl.reg, &reg_ctrl.data);
	if (r)
		return r;

	if (copy_to_user((int __user *)ifr->ifr_ifru.ifru_data, &reg_ctrl,
			 sizeof(reg_ctrl)))
		return -EFAULT;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
static int f81605_ndo_siocdevprivate(struct net_device *dev, struct ifreq *ifr,
				     void __user *data, int cmd)
{
	switch (cmd) {
	case FINTEK_IOCTL_WR_REG:
		return f81605_ioctl_write(dev, ifr);

	case FINTEK_IOCTL_RD_REG:
		return f81605_ioctl_read(dev, ifr);
	}

	return -EOPNOTSUPP;
}
#else
static int f81605_ndo_do_ioctl(struct net_device *dev, struct ifreq *ifr,
			       int cmd)
{
	switch (cmd) {
	case FINTEK_IOCTL_WR_REG:
		return f81605_ioctl_write(dev, ifr);

	case FINTEK_IOCTL_RD_REG:
		return f81605_ioctl_read(dev, ifr);
	}

	return -ENOTSUPP;
}
#endif

static const struct net_device_ops f81605_netdev_ops = {
	.ndo_open = f81605_open,
	.ndo_stop = f81605_close,
	.ndo_start_xmit = f81605_start_xmit,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	.ndo_change_mtu = can_change_mtu,
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	.ndo_siocdevprivate = f81605_ndo_siocdevprivate,
#else
	.ndo_do_ioctl = f81605_ndo_do_ioctl,
#endif
};

#if 0
static int f81605_ram_save(struct usb_interface *intf)
{
	struct f81605_priv *priv;
	struct usb_device *dev;
	int i, r;

	dev = interface_to_usbdev(intf);
	priv = usb_get_intfdata(intf);

	for (i = 0; i < ARRAY_SIZE(priv->ram_info); ++i) {
		r = f81605_get_register(dev, 0x400 + i, &priv->ram_info[i]);
		if (r)
			return r;
	}

	return 0;
}
#endif

static void f81605_ram_restore(struct usb_interface *intf)
{
	struct f81605_priv *priv;
	struct usb_device *dev;
	int i, r;
	const unsigned char custom_eeprom[] = {
		0x55, 0xAA, 0x02, 0x42, 0x2C, 0x02, 0x04, 0x21, 0x02, 0x00,
		0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	dev = interface_to_usbdev(intf);
	priv = usb_get_intfdata(intf);

	for (i = 0; i < ARRAY_SIZE(custom_eeprom); ++i) {
		r = f81605_set_register(dev, 0x400 + i, custom_eeprom[i]);
		if (r)
			break;
	}
}

static int f81605_reboot_notify(struct notifier_block *nb, unsigned long code,
				void *unused)
{
	struct f81605_priv *priv;

	priv = container_of(nb, struct f81605_priv, reboot_notifier);

	f81605_ram_restore(priv->intf);

	return NOTIFY_DONE;
}

static ssize_t gpio_ctrl_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct f81605_port_priv *port_priv;
	struct f81605_priv *priv;
	struct net_device *netdev;
	struct usb_device *usbdev;
	int status;
	u8 data;

	if (count < 1)
		return -EINVAL;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);
	usbdev = port_priv->dev;

	mutex_lock(&priv->mutex);

	do {
		status = f81605_get_register(usbdev, F81605_GPIO_DATA_REG,
					     &data);
		if (status)
			break;

		if (buf[0] == '1')
			data |= BIT(netdev->dev_id);
		else
			data &= ~BIT(netdev->dev_id);

		status = f81605_set_register(usbdev, F81605_GPIO_DATA_REG,
					     data);
		if (status)
			break;

		pr_debug("%s: data: %xh\n", __func__, data);
	} while (0);

	mutex_unlock(&priv->mutex);

	if (status)
		return -EINVAL;

	return count;
}

static ssize_t gpio_ctrl_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct f81605_port_priv *port_priv;
	struct f81605_priv *priv;
	struct net_device *netdev;
	struct usb_device *usbdev;
	u8 mode, data;
	int status;

	netdev = container_of(dev, struct net_device, dev);
	port_priv = netdev_priv(netdev);
	priv = usb_get_intfdata(port_priv->intf);
	usbdev = port_priv->dev;

	mutex_lock(&priv->mutex);
	//status = f81605_get_register(usbdev, F81605_GPIO_MODE_REG, &mode);
	status = f81605_get_register(usbdev, F81605_GPIO_DATA_REG, &data);
	mutex_unlock(&priv->mutex);

	if (status)
		return -EINVAL;

	pr_debug("%s: mode: %xh, data: %xh\n", __func__, mode, data);

	return sprintf(buf, "%d\n", !!(data & BIT(netdev->dev_id)));
}

static DEVICE_ATTR_RW(gpio_ctrl);

static int f81605_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(intf);
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *epd;
	struct f81605_priv *priv;
	struct f81605_port_priv *port_priv;
	struct workqueue_struct *wq;
	int i, err, dev_count, max_p;
	u8 ver, tmp;

	dev_info(&intf->dev, "%s: Fintek F81605 driver version: %s\n",
		 __func__, DRV_VER);

	dev_count = max_p = 0;
	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		epd = &iface_desc->endpoint[i].desc;
		if (usb_endpoint_is_bulk_out(epd)) {
			dev_count++;

			if (usb_endpoint_maxp(epd) > max_p)
				max_p = usb_endpoint_maxp(epd);
		}
	}

	if (dev_count != 1 && dev_count != 2) {
		dev_err(&intf->dev, "%s: dev_count: %d failed\n", __func__,
			dev_count);
		return -ENODEV;
	}

	priv = devm_kzalloc(&intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);
	priv->dev_count = dev_count;
	priv->intf = intf;

	usb_set_intfdata(intf, priv);

	/* can1/can2 soft reset */
	err = f81605_set_register(dev, 0x10c, 0x03);
	if (err)
		return err;

	msleep(10);

	// enable led & loading func
	err = f81605_set_register(dev, 0x10c, BIT(6));
	if (err)
		return err;

	err = f81605_get_register(dev, F81605_HW_VERSION, &ver);
	if (err)
		return err;

	if (max_p == 64)
		tmp = BIT(0); // max rx packet 64B
	else
		tmp = 0; // max rx packet 512B

	err = f81605_set_mask_register(dev, 0x104, BIT(0), tmp);
	if (err)
		return err;

	for (i = 0; i < priv->dev_count; ++i) {
		wq = create_workqueue(DRV_NAME);
		if (!wq) {
			dev_err(&intf->dev, "%s: alloc wq%d err\n", __func__,
				i);
			continue;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
		priv->netdev[i] =
			alloc_candev(sizeof(*port_priv), F81605_MAX_TX_URBS);
#else
		priv->netdev[i] = alloc_candev(sizeof(*port_priv));
#endif
		if (!priv->netdev[i]) {
			dev_err(&intf->dev, "Couldn't alloc candev: %d\n", i);
			return -ENOMEM;
		}

		port_priv = netdev_priv(priv->netdev[i]);
		priv->netdev[i]->dev_id = i;

		spin_lock_init(&port_priv->lock);

#if CAN_TIMER_DBG
		INIT_DELAYED_WORK(&port_priv->delayed_timer_work,
				  f81605_delayed_timer_work);
#endif

#if CAN_DELAYED_RX_DBG
		INIT_DELAYED_WORK(&port_priv->delayed_rx_work,
				  f81605_delayed_rx_work);
#endif

#if CAN_STS_POLL
		INIT_DELAYED_WORK(&port_priv->delayed_sts_work,
				  f81605_delayed_sts_work);
#endif

		INIT_DELAYED_WORK(&port_priv->delayed_oneshot_work,
				  f81605_oneshot_work);

		if (disable_canfd)
			port_priv->is_disable_fd = true;
		else
			port_priv->is_disable_fd = false;

		port_priv->wq = wq;
		port_priv->intf = intf;
		port_priv->dev = dev;
		port_priv->ver = ver;
		port_priv->ocr = OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL;
		port_priv->cdr = CDR_CBP;
		port_priv->can.state = CAN_STATE_STOPPED;
		port_priv->can.clock.freq = 80000000;

		port_priv->can.bittiming_const = &f81605_bittiming_const;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0)
		port_priv->can.fd.data_bittiming_const =
			&f81605_data_bittiming_const;
#else
		port_priv->can.data_bittiming_const =
			&f81605_data_bittiming_const;
#endif

		port_priv->can.do_set_bittiming = f81605_set_bittiming;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0)
		port_priv->can.fd.do_set_data_bittiming =
			f81605_set_data_bittiming;
#else
		port_priv->can.do_set_data_bittiming =
			f81605_set_data_bittiming;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 16, 0)
		//port_priv->can.bitrate_max = 5000000;
#endif

		port_priv->can.do_set_mode = f81605_set_mode;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		port_priv->can.do_get_berr_counter = f81605_get_berr_counter;
		port_priv->can.ctrlmode_supported =
			CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY |
			CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_ONE_SHOT |
			CAN_CTRLMODE_BERR_REPORTING;

		if (!port_priv->is_disable_fd) {
			port_priv->can.ctrlmode_supported |=
				CAN_CTRLMODE_FD | CAN_CTRLMODE_FD_NON_ISO;
		}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
		port_priv->can.ctrlmode_supported |= CAN_CTRLMODE_PRESUME_ACK;
#endif
		priv->netdev[i]->netdev_ops = &f81605_netdev_ops;
		priv->netdev[i]->flags |= IFF_ECHO; /* we support local echo */

		SET_NETDEV_DEV(priv->netdev[i], &intf->dev);

		err = f81605_prepare_urbs(priv->netdev[i]);
		if (err) {
			/* todo error */

			dev_err(&intf->dev, "%s: Couldn't prepare urbs: %d\n",
				__func__, err);
			return err;
		}

		err = register_candev(priv->netdev[i]);
		if (err) {
			netdev_err(priv->netdev[i],
				   "couldn't register CAN device: %d\n", err);
			return err;
		}

		port_priv->netdev = priv->netdev[i];

		device_create_file(&priv->netdev[i]->dev, &dev_attr_gpio_ctrl);

		dev_info(&intf->dev, "Channel #%d registered as %s\n", i,
			 priv->netdev[i]->name);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
		devm_can_led_init(priv->netdev[i]);
#endif

		device_create_file(&port_priv->netdev->dev, &dev_attr_ssp_val);
		device_create_file(&port_priv->netdev->dev,
				   &dev_attr_bus_loading);

		if (reg_debug) {
			device_create_file(&port_priv->netdev->dev,
					   &dev_attr_reg_dump_all);
			device_create_file(&port_priv->netdev->dev,
					   &dev_attr_reg_dump);
		}
	}

	priv->reboot_notifier.notifier_call = f81605_reboot_notify;
	err = register_reboot_notifier(&priv->reboot_notifier);
	if (err) {
		dev_err(&intf->dev, "failed to register reboot notifier: %d\n",
			err);

		return err;
	}

	/* todo error recovery */
	return 0;
}

/* Called by the usb core when driver is unloaded or device is removed */
static void f81605_disconnect(struct usb_interface *intf)
{
	struct f81605_priv *priv = usb_get_intfdata(intf);
	struct f81605_port_priv *port_priv;
	int i;

	unregister_reboot_notifier(&priv->reboot_notifier);
	f81605_ram_restore(priv->intf);

	for (i = 0; i < priv->dev_count; ++i) {
		if (!priv->netdev[i])
			continue;

		netdev_info(priv->netdev[i], "device disconnected\n");

		f81605_remove_urbs(priv->netdev[i]);

		device_remove_file(&priv->netdev[i]->dev, &dev_attr_ssp_val);
		device_remove_file(&priv->netdev[i]->dev,
				   &dev_attr_bus_loading);

		if (reg_debug) {
			device_remove_file(&priv->netdev[i]->dev,
					   &dev_attr_reg_dump_all);
			device_remove_file(&priv->netdev[i]->dev,
					   &dev_attr_reg_dump);
		}

		device_remove_file(&priv->netdev[i]->dev, &dev_attr_gpio_ctrl);

		unregister_netdev(priv->netdev[i]);

		port_priv = netdev_priv(priv->netdev[i]);

		drain_workqueue(port_priv->wq);
		destroy_workqueue(port_priv->wq);

		free_candev(priv->netdev[i]);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
static void f81605_shutdow(struct usb_interface *intf)
{
	f81605_ram_restore(intf);
}
#endif

static int f81605_prepare_suspend(struct usb_interface *intf, int idx)
{
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	struct f81605_priv *priv_intf;
	int i;

	if (idx >= F81605_MAX_DEV)
		return -ENODEV;

	priv_intf = usb_get_intfdata(intf);
	netdev = priv_intf->netdev[idx];
	priv = netdev_priv(netdev);

	netif_stop_queue(netdev);

	for (i = 0; i < F81605_MAX_TX_URBS; ++i)
		usb_poison_urb(priv->write_urb[i]);

	for (i = 0; i < F81605_MAX_RX_URBS; ++i) {
		usb_poison_urb(priv->read_rx_urb[i]);
#if !CAN_STS_POLL
		usb_poison_urb(priv->read_sts_urb[i]);
#endif
	}

#if CAN_STS_POLL
	cancel_delayed_work_sync(&priv->delayed_sts_work);
#endif

	return f81605_get_sja1000_register(priv->dev, netdev->dev_id,
					   SJA1000_MOD, &priv->old_mod);
}

static int f81605_prepare_resume(struct usb_interface *intf, int idx)
{
	struct f81605_port_priv *priv;
	struct net_device *netdev;
	struct f81605_priv *priv_intf;
	int i, r;

	if (idx >= F81605_MAX_DEV)
		return -ENODEV;

	priv_intf = usb_get_intfdata(intf);
	netdev = priv_intf->netdev[idx];
	priv = netdev_priv(netdev);

	for (i = 0; i < F81605_MAX_TX_URBS; ++i) {
		usb_unpoison_urb(priv->write_urb[i]);
		set_bit(i, &priv->write_urb_sts);
	}

	for (i = 0; i < F81605_MAX_RX_URBS; ++i) {
		usb_unpoison_urb(priv->read_rx_urb[i]);
#if !CAN_STS_POLL
		usb_unpoison_urb(priv->read_sts_urb[i]);
#endif
	}

	f81605_register_urbs(netdev);

	r = f81605_set_sja1000_register(priv->dev, netdev->dev_id, SJA1000_MOD,
					priv->old_mod);
	if (r) {
		dev_info(&intf->dev, "%s: set old_mod failed: %x\n", __func__,
			 r);
		return r;
	}

	if (!(priv->old_mod & MOD_RM)) {
		netif_wake_queue(netdev);

#if CAN_STS_POLL
		queue_delayed_work(priv->wq, &priv->delayed_sts_work,
				   CAN_STS_POLL_TIMER);
#endif
	}

	return 0;
}

static int f81605_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct f81605_priv *priv;
	int i, r;

	dev_info(&intf->dev, "%s\n", __func__);

	priv = usb_get_intfdata(intf);

	for (i = 0; i < priv->dev_count; ++i) {
		r = f81605_prepare_suspend(intf, i);
		if (r)
			return r;
	}

	return 0;
}

static int f81605_resume(struct usb_interface *intf)
{
	struct f81605_priv *priv;
	int i, r;

	dev_info(&intf->dev, "%s\n", __func__);

	priv = usb_get_intfdata(intf);

	for (i = 0; i < priv->dev_count; ++i) {
		r = f81605_prepare_resume(intf, i);
		if (r)
			return r;
	}

	return 0;
}

static struct usb_driver f81605_driver = {
	.name = DRV_NAME,
	.probe = f81605_probe,
	.disconnect = f81605_disconnect,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
	.shutdown = f81605_shutdow,
#endif
	.suspend = f81605_suspend,
	.resume = f81605_resume,
	.id_table = F81605_table,
};

module_usb_driver(f81605_driver);

MODULE_AUTHOR("Peter Hong <Peter_Hong@fintek.com.tw>");
MODULE_DESCRIPTION("Fintek F81605 USB to 2xCANFD");
MODULE_LICENSE("GPL v2");
