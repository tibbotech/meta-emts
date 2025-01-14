FILESEXTRAPATHS:prepend := "${THISDIR}/linux-sp-5.4:"

SRC_URI:append:tppg2 = " file://dts/sp7021-emtsH-tppg2.dts.patch"
SRC_URI:append:tppg2 = " file://dts/sp7021-emtsG-tppg2.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emtsH.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emtsG.dts.patch"

KERNEL_DEVICETREE:append:tppg2 = " sp7021-emtsH-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2 = " sp7021-emtsG-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emtsH.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emtsG.dtb"
