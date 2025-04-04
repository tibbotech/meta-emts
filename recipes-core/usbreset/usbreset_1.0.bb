DESCRIPTION = "USB device reset program"
HOMEPAGE = "http://marc.info/?l=linux-usb&m=121459435621262&w=2"
MAINTAINER = "Dmitry Dvorkin <dvorkin@tibbo.com>"
LICENSE = "GPL-3.0-or-later"
SECTION = "console/utils"
PACKAGES = "${PN}-dbg ${PN}"
SRCREV="${AUTOREV}"

inherit autotools autotools-brokensep

SRC_URI  = "file://usbreset.c"
SRC_URI += "file://Makefile"

S = "${WORKDIR}"

FILES:${PN} += "/home/root/usbreset"

LIC_FILES_CHKSUM = "file://${FILESDIR_tibbo}/common-licenses/GPL-3.0-or-later;md5=1c76c4cc354acaac30ed4d5eefea7245"
