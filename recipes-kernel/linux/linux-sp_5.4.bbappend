FILESEXTRAPATHS:prepend := "${THISDIR}/linux-sp-5.4:"

SRC_URI:append = " file://kmeta-emts;type=kmeta;name=kmeta-emts;destsuffix=kmeta-emts"

SRC_URI:append:tppg2 = " file://dts/sp7021-emtsH-tppg2.dts.patch"
SRC_URI:append:tppg2 = " file://dts/sp7021-emtsG-tppg2.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emtsH.dts.patch"
SRC_URI:append:tppg2-emts = " file://dts/sp7021-emtsG.dts.patch"

# tst
SRC_URI:append:tppg2 = " file://hid-debug.c.patch"
SRC_URI:append:tppg2 = " file://input-event-codes.h.patch"
SRC_URI:append:tppg2 = " file://xpad.c-66.2.patch"

KERNEL_DEVICETREE:append:tppg2 = " sp7021-emtsH-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2 = " sp7021-emtsG-tppg2.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emtsH.dtb"
KERNEL_DEVICETREE:append:tppg2-emts = " sp7021-emtsG.dtb"

KERNEL_FEATURES:append = " joystick/xpad.scc"
KERNEL_FEATURES:append = " debug_otg.scc"
