DESCRIPTION = "Fintek FB1605 USB-to-CANBUS module"
MAINTAINER = "Dvorkin Dmitry <dvorkin@tibbo.com>"
SECTION = "kernel/modules"
PRIORITY = "optional"
LICENSE = "GPL-2.0-only"
PR = "r1"

PACKAGES =+ "${PN}-includes"
PACKAGE_ARCH:${PN}-includes = "all"

# src in /files/kernel-mod-fintek/*
#S = "${WORKDIR}/${PN}"
S = "${WORKDIR}"

inherit module

SRC_URI  = "file://f81605.c"
SRC_URI += "file://Makefile"
SRC_URI += "file://sja1000.h"

do_install() {
 install -m 0755 -d ${D}${base_libdir}/modules/${KERNEL_VERSION}/kernel/3rdparty
 install -m 0644 ${S}/f81605${KERNEL_OBJECT_SUFFIX} ${D}${base_libdir}/modules/${KERNEL_VERSION}/kernel/3rdparty
}

LIC_FILES_CHKSUM = "file://${FILESDIR_tibbo}/common-licenses/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"
