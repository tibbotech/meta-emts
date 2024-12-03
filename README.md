# meta-emts

Public Yocto layer of EMTS sp7021-based project.

## Dependencies

https://github.com/tibbotech/meta-tibbo
    tibbo

https://git.openembedded.org/openembedded-core
    Core
    networking-layer
    multimedia-layer
    webserver

## Quick links

* Git repo: https://github.com/tibbotech/repo-manifests

## Description

Layer contains private apps, DTSes and image definition for sp7021-based board.

## Quick Start

### Tibbo Layers Setup
```
curl https://raw.githubusercontent.com/tibbotech/repo-manifests/master/clone.sh > ./clone.sh && chmod 0755 ./clone.sh && ./clone.sh
repo3 sync
TEMPLATECONF=`pwd`/layers/meta-tibbo/conf/templates/tppg2 . layers/openembedded-core/oe-init-build-env ./build.tppg2
install -m 0644 ../layers/meta-tibbo/conf/templates/site.conf conf/
```
### + This Layer
```
git clone git@github.com:tibbotech/meta-emts.git ../layers/meta-emts
MACHINE=tppg2 bitbake-layers add-layer ../layers/meta-emts
```

### Append to your local.conf
```
ISP_VAR_DTB = "sp7021-emts.dtb"
```

### Building
```
MACHINE=tppg2 bitbake mc:tppg2:img-emts
```
ISPBOOOT.BIN will be placed at BUILDDIR/deploy/images/tppg2/emmc0/

## Maintainers

* Dvorkin Dmitry `<dvorkin at tibbo.com>`

Read [HOWTO.md](HOWTO.md) description to see how to adopt it.
