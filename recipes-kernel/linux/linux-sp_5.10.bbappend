FILESEXTRAPATHS:prepend := "${THISDIR}/linux-sp-5.10:"

SRC_URI:append = " file://kmeta-emts;type=kmeta;name=kmeta-emts;destsuffix=kmeta-emts"

# for EV: disable NAND, NOR
SRC_URI:append:tppg2 = " file://sp7021-ev.dts.noNOR.patch"

SRC_URI:append:tppg2 = " file://dts/sp7021-emtsH-tppg2.dts.patch"
SRC_URI:append:tppg2 = " file://dts/sp7021-emtsG-tppg2.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emtsH.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emtsG.dts.patch"

KERNEL_DEVICETREE:append:tppg2 = " sp7021-emtsH-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2 = " sp7021-emtsG-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emtsH.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emtsG.dtb"

KERNEL_FEATURES:append = " joystick/xpad.scc"
KERNEL_FEATURES:append = " debug_otg.scc"
