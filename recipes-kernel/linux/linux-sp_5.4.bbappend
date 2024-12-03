FILESEXTRAPATHS:prepend := "${THISDIR}/linux-sp-5.4:"

SRC_URI:append:tppg2 = " file://dts/sp7021-emts-tppg2.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emts.dts.patch"

KERNEL_DEVICETREE:append:tppg2 = " sp7021-emts-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emts.dtb"
