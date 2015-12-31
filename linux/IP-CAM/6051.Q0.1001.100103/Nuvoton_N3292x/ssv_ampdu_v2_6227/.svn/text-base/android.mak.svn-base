KBUILD_TOP := $(ANDROID_BUILD_TOP)/kernel/drivers/net/wireless/ssv
include $(KBUILD_TOP)/config.mak

KMODULE_NAME=ssv6200

#KERNEL_MODULES := ssvdevice
KERN_SRCS := ssvdevice/ssvdevice.c
KERN_SRCS += ssvdevice/ssv_cmd.c
#KERNEL_MODULES += hci
KERN_SRCS += hci/ssv_hci.c
#KERNEL_MODULES += ssv6200smac
KERN_SRCS += ssv6200smac/init.c
KERN_SRCS += ssv6200smac/dev.c
KERN_SRCS += ssv6200smac/ssv_rc.c
KERN_SRCS += ssv6200smac/ssv_ht_rc.c
KERN_SRCS += ssv6200smac/ap.c
KERN_SRCS += ssv6200smac/ampdu.c
KERN_SRCS += ssv6200smac/ssv6xxx_debugfs.c
KERN_SRCS += ssv6200smac/sec.c
#KERNEL_MODULES += hwif/sdio
KERN_SRCS += hwif/sdio/sdio.c
#KERNEL_MODULES += crypto
#KERN_SRCS += crypto/aes_glue.c
#KERN_SRCS += crypto/aes-armv4.S
#KERN_SRCS += crypto/sha1_glue.c
#KERN_SRCS += crypto/sha1-armv4-large.S
#KERNEL_MODULES += platform
KERN_SRCS += platform/generic_wlan.c

$(KMODULE_NAME)-y += $(KERN_SRCS:.c=.o)
#$(KMODULE_NAME)-y += $(KERN_SRCS:.S=.o)
obj-$(CONFIG_SSV6200_CORE) += $(KMODULE_NAME).o

KERNEL_SOURCE =$(ANDROID_BUILD_TOP)/kernel
KBUILD_DIR := $(KBUILD_TOP)

all:
	@$(MAKE) ARCH=arm CROSS_COMPILE=$(ARM_EABI_TOOLCHAIN)/arm-eabi- \
		-C $(KERNEL_SOURCE) \
                SUBDIRS=$(KBUILD_DIR) CONFIG_DEBUG_SECTION_MISMATCH=y \
                modules
clean:
	@$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(KBUILD_DIR) clean
