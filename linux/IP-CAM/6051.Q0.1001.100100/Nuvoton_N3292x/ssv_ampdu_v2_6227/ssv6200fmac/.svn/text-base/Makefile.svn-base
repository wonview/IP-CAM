obj-m     += ssv6200fmac.o
obj-m     += ssv6200fmac_sdio.o

ssv6200fmac-y += cfg.o
ssv6200fmac-y += cmd.o
ssv6200fmac-y += cmdresp.o
ssv6200fmac-y += debugfs.o
ssv6200fmac-y += ethtool.o
ssv6200fmac-y += main.o
ssv6200fmac-y += rx.o
ssv6200fmac-y += tx.o
ssv6200fmac-y += cabrio-if/cabrio_host_utils.o

ssv6200fmac_sdio-y += ssv_sdio.o

#Define CONFIG_CABRIO_DEBUG to show debug messages
ccflags-y += -DCONFIG_CABRIO_DEBUG

ifndef ($(KBUILD_EXTMOD),)
KDIR=/lib/modules/`uname -r`/build

_all:
	$(MAKE) -C $(KDIR) M=$(PWD) KBUILD_EXTRA_SYMBOLS=$(PWD)/../ssvdevice/Module.symvers modules 2>&1 | tee make.log
	
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f make.log
	
install:
	@-rmmod ssv6200fmac_sdio
	@-rmmod ssv6200fmac
	$(MAKE) INSTALL_MOD_DIR=kernel/drivers/net/wireless/ssv6200 -C $(KDIR) M=$(PWD) modules_install
	modprobe ssv6200fmac_sdio

endif
