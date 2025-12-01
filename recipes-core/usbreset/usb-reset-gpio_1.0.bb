DESCRIPTION = "USB device reset program"
MAINTAINER = "Dmitry Dvorkin <dvorkin@tibbo.com>"
LICENSE = "GPL-3.0-or-later"
SECTION = "console/utils"
SRCREV="${AUTOREV}"

inherit systemd allarch

SRC_URI  = "file://usb_reset_gpio.sh"
SRC_URI += "file://usb-reset-gpio.service"

S = "${WORKDIR}"

do_install() {
 install -d ${D}/home/root
 install -m 0755 ${WORKDIR}/usb_reset_gpio.sh ${D}/home/root/
 install -d ${D}${systemd_system_unitdir}/
 install -m 0644 ${WORKDIR}/usb-reset-gpio.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += "/home/root/usb_reset_gpio.sh"

SYSTEMD_SERVICE:${PN} = "usb-reset-gpio.service"

LIC_FILES_CHKSUM = "file://${FILESDIR_tibbo}/common-licenses/GPL-3.0-or-later;md5=1c76c4cc354acaac30ed4d5eefea7245"
