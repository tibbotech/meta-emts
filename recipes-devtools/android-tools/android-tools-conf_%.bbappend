FILESEXTRAPATHS:prepend := "${THISDIR}/${BPN}:"

inherit systemd

CONFFILES:${PN} += " ${sysconfdir}/modules-load.d/adbd.conf"

SRC_URI += "file://g_ffs.conf"
SRC_URI += "file://g_ffs.conf.modprobe"
#SRC_URI += "file://dev-usb-ffs-adb.mount"
SRC_URI += "file://adbd.service"

do_install:append() {
 install -d ${D}${sysconfdir}/modules-load.d
 install -m 0644 ${WORKDIR}/g_ffs.conf ${D}${sysconfdir}/modules-load.d/
 install -d ${D}${sysconfdir}/modprobe.d/
 install -m 0644 ${WORKDIR}/g_ffs.conf.modprobe ${D}${sysconfdir}/modprobe.d/g_ffs.conf
 install -d ${D}${systemd_unitdir}/system
# install -m 0644 ${WORKDIR}/dev-usb-ffs-adb.mount ${D}${systemd_unitdir}/system/
 install -m 0644 ${WORKDIR}/adbd.service ${D}${systemd_unitdir}/system/
}

FILES:${PN} += "${systemd_unitdir}/system/adbd.service"

#SYSTEMD_SERVICE:${PN} += "dev-usb-ffs-adb.mount"
SYSTEMD_SERVICE:${PN} += "adbd.service"
