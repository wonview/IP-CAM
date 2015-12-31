KBUILD_TOP := $(PWD)
include $(KBUILD_TOP)/customer.cfg
include $(KBUILD_TOP)/ssv_ampdu_v2_$(DRV_VER)/config.mak
KMODULE_NAME=ssv6051
EXTRA_CFLAGS := -I$(KBUILD_TOP)/ssv_ampdu_v2_$(DRV_VER) -I$(KBUILD_TOP)/ssv_ampdu_v2_$(DRV_VER)/include
KERNEL_MODULES := ssv_ampdu_v2_$(DRV_VER)/ssvdevice
KERN_SRCS := ssv_ampdu_v2_$(DRV_VER)/ssvdevice/ssvdevice.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssvdevice/ssv_cmd.c
KERNEL_MODULES += ssv_ampdu_v2_$(DRV_VER)/hci
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/hci/ssv_hci.c
KERNEL_MODULES += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/init.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/dev.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/ssv_rc.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/ssv_ht_rc.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/ap.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/ampdu.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/ssv6xxx_debugfs.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/sec.c
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/ssv6200smac/efuse.c
KERNEL_MODULES += ssv_ampdu_v2_$(DRV_VER)/hwif/sdio
KERN_SRCS += ssv_ampdu_v2_$(DRV_VER)/hwif/sdio/sdio.c
#KERNEL_MODULES += crypto
#KERN_SRCS += crypto/aes_glue.c
#KERN_SRCS += crypto/aes-armv4.S
#KERN_SRCS += crypto/sha1_glue.c
#KERN_SRCS += crypto/sha1-armv4-large.S
#KERNEL_MODULES += platform
KERN_SRCS += generic_wlan.c

$(KMODULE_NAME)-y += $(KERN_SRCS:.c=.o)
#$(KMODULE_NAME)-y += $(KERN_SRCS:.S=.o)
obj-$(CONFIG_SSV6200_CORE) += $(KMODULE_NAME).o

KBUILD_DIR := $(KBUILD_TOP)
CROSS := /usr/local/arm_linux_4.2/bin/arm-linux-
KERNEL_SOURCE := ../linux-2.6.35.4/
all:
	@$(MAKE) ARCH=arm CROSS_COMPILE=$(CROSS) \
		-C $(KERNEL_SOURCE) \
                SUBDIRS=$(KBUILD_DIR) CONFIG_DEBUG_SECTION_MISMATCH=y \
                modules
clean:
	@for dir in $(KERNEL_MODULES) ; do \
            $(MAKE) -C $$dir clean KBUILD_DIR=$(PWD)/$$dir ; \
        done
	@$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(KBUILD_DIR) clean
