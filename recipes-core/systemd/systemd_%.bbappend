FILESEXTRAPATHS:prepend := "${THISDIR}/${BPN}:"

CONFFILES:${PN} += " ${sysconfdir}/systemd/network/can0.network"
CONFFILES:${PN} += " ${sysconfdir}/systemd/network/can1.network"

SRC_URI += "file://can0.network"
SRC_URI += "file://can1.network"

do_install:append() {
 install -m 0644 ${WORKDIR}/can0.network ${D}${sysconfdir}/systemd/network/can0.network
 install -m 0644 ${WORKDIR}/can1.network ${D}${sysconfdir}/systemd/network/can1.network
}
