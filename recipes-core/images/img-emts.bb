DESCRIPTION = "EMTS image"

LICENSE = "MIT"

require recipes-core/images/inc-p-123.inc
require recipes-core/images/inc-rootfs.inc
require recipes-core/images/inc-tps-gen.inc

IMAGE_FEATURES = ""

IMAGE_FEATURES += "ssh-server-openssh"
IMAGE_INSTALL  += "openssh-sftp-server"
IMAGE_FEATURES += "package-management"

IMAGE_INSTALL += "rsync"

IMAGE_INSTALL += "gdbserver"
#IMAGE_INSTALL += "tcf-agent"

IMAGE_INSTALL += "tzdata-asia"

#IMAGE_INSTALL += "tps-resetconf"
#IMAGE_INSTALL += "wireguard-tools"

IMAGE_INSTALL += "tps-dts"
IMAGE_INSTALL += "iproute2"
IMAGE_INSTALL += "usbutils"
IMAGE_INSTALL += "inputattach joystick-jscal joystick"
# for adroid-tools
#IMAGE_INSTALL += "android-tools"
#PREFERRED_PROVIDER_android-tools-conf = "android-tools-conf-configfs"
IMAGE_INSTALL += "android-tools-adbd"
IMAGE_INSTALL += "kernel-modules"

# temporary there
IMAGE_INSTALL += "devmem2 ethtool mtd-utils mmc-utils"
IMAGE_INSTALL += "mc strace pstree lsof"

#IMAGE_INSTALL += "kernel-module-xone usbreset"
IMAGE_INSTALL += "usbreset"

# test
TOOLCHAIN_TARGET_TASK += "kernel-devsrc"
# test
TOOLCHAIN_TARGET_TASK += "libusb1"

# tmp
#IMAGE_INSTALL += "dkms kernel-devsrc packagegroup-core-buildessential"
