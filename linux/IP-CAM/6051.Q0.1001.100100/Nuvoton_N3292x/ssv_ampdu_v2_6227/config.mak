
KVERSION="`uname -r`"
#DRVPATH=/lib/modules/$(KVERSION)/kernel/drivers/net/wireless/ssv6200
DRVPATH=kernel/drivers/net/wireless/ssv6200
KCFLAG += -Werror
EXTRA_CFLAGS := -I$(KBUILD_TOP) -I$(KBUILD_TOP)/include




CONFIG_SSV6200_CORE=m
CONFIG_SSV6200_SDIO=m
CONFIG_SSV6200_DEVICE_TYPE=m



CONFIG_SSV_COMMON=m
CONFIG_SSV_DEBUG=y

ccflags-y += -Werror

############################################################
# If you change the settings, please change the file synchronization
# ssv6200smac\firmware\include\config.h & compiler firmware
############################################################
#ccflags-y += -DCONFIG_SSV_CABRIO_A
ccflags-y += -DCONFIG_SSV_CABRIO_E

#PADPD
#ccflags-y += -DCONFIG_SSV_DPD

ccflags-y += -DCONFIG_SSV_COMMON
ccflags-y += -DCONFIG_SSV_DEBUG
ccflags-y += -DCONFIG_SSV_DEBUG_DBG_MSG
#ccflags-y += -DCONFIG_SSV_CABRIO_MB_DEBUG
ccflags-y += -DCONFIG_SSV6XXX_DEBUGFS

#SDIO
#ccflags-y += -DCONFIG_SSV_TX_LOWTHRESHOLD

############################################################
# Rate control update for MPDU.
############################################################
ccflags-y += -DRATE_CONTROL_REALTIME_UPDATA

############################################################
# NOTE:
#    Only one of the following flags could be turned on.
# It also turned off the following flags. In this case, 
# pure software security or pure hardware security is used.
#
############################################################
#ccflags-y += -DCONFIG_SSV_SW_ENCRYPT_HW_DECRYPT
#ccflags-y += -DCONFIG_SSV_HW_ENCRYPT_SW_DECRYPT


#workaround
#ccflags-y += -DCONFIG_SSV_CABRIO_EXT_PA

#CONFIG_SSV_SDIO_BRIDGE=y
CONFIG_SSV_CABRIO_WIFI=y
CONFIG_SSV_SDIO_BRIDGE_HW=m
CONFIG_SSV_SDIO_BRIDGE_DEBUGFS=y
CONFIG_SSV_CABRIO_HW=m
CONFIG_SSV_CABRIO_COMMON=m
CONFIG_SSV_CABRIO_BTCOEX_SUPPORT=y
CONFIG_SSV_CABRIO=m
CONFIG_SSV_CABRIO_SDIO=m
CONFIG_SSV_CABRIO_DEBUGFS=y
CONFIG_SSV_CABRIO_MAC_DEBUG=y
CONFIG_SSV_CABRIO_RATE_CONTROL=y
#CONFIG_SSV_SUPPORT_AW_SUNXI=y
#CONFIG_SSV_SUPPORT_SCX35=y
#CONFIG_SSV_SUPPORT_SP8825EA=y
#CONFIG_SSV_SUPPORT_SP6821A=y
#CONFIG_SSV_SUPPORT_ANDROID=y
#CONFIG_SSV_SUPPORT_BTCX=y

# Use ssv.ko as SDIO driver entry
#CONFIG_CABRIO_DEPENDS_ON_SSV_DEVICE=n

# FOR WFA
#ccflags-y += -DWIFI_CERTIFIED

#ccflags-y += -DCONFIG_SSV_SDIO_EXT_INT

#ccflags-y += -DCONFIG_SSV_SDIO_BRIDGE
ccflags-y += -DCONFIG_SSV_CABRIO_WIFI
ccflags-y += -DCONFIG_SSV_SDIO_BRIDGE_HW
ccflags-y += -DCONFIG_SSV_SDIO_BRIDGE_DEBUGFS
ccflags-y += -DCONFIG_SSV_CABRIO_HW
ccflags-y += -DCONFIG_SSV_CABRIO_COMMON
ccflags-y += -DCONFIG_SSV_CABRIO_BTCOEX_SUPPORT
ccflags-y += -DCONFIG_SSV_CABRIO
ccflags-y += -DCONFIG_SSV_CABRIO_SDIO
ccflags-y += -DCONFIG_SSV_CABRIO_DEBUGFS
ccflags-y += -DCONFIG_SSV_CABRIO_MAC_DEBUG
ccflags-y += -DCONFIG_SSV_CABRIO_RATE_CONTROL


ccflags-y += -DCONFIG_SSV6200_CLI_ENABLE

ccflags-$(CONFIG_SSV_SUPPORT_AW_SUNXI) += -DCONFIG_SSV_SUPPORT_AW_SUNXI
ccflags-$(CONFIG_SSV_SUPPORT_AW_SUNXI) += -DCONFIG_FW_ALIGNMENT_CHECK
ccflags-$(CONFIG_SSV_SUPPORT_SCX35)    += -DCONFIG_SSV_SUPPORT_SCX35
ccflags-$(CONFIG_SSV_SUPPORT_SP8825EA) += -DCONFIG_SSV_SUPPORT_SP8825EA
#ccflags-y += -DCONFIG_SSV_SUPPORT_SP6821A
ccflags-$(CONFIG_SSV_SUPPORT_ANDROID)  += -DCONFIG_SSV_SUPPORT_ANDROID
ccflags-$(CONFIG_SSV_SUPPORT_BTCX)  += -DCONFIG_SSV_SUPPORT_BTCX

#ccflags-y += -DCONFIG_SSV6200_CORE
#ccflags-y += -DCONFIG_SSV6200_SDIO
#ccflags-y += -DCONFIG_SSV6200_DEVICE_TYPE

#ccflags-y += -DUSE_GENERIC_DECI_TBL

ccflags-y += -DCONFIG_SSV6200_HAS_RX_WORKQUEUE
ccflags-y += -DUSE_THREAD_RX
ccflags-y += -DUSE_THREAD_TX
ccflags-y += -DENABLE_AGGREGATE_IN_TIME
ccflags-y += -DENABLE_INCREMENTAL_AGGREGATION
#ccflags-y += -DUSE_LOCAL_CRYPTO
#ccflags-y += -DMULTI_THREAD_ENCRYPT
#ccflags-y += -DUSE_BATCH_RX
#ccflags-y += -DCONFIG_IRQ_DEBUG_COUNT
#ccflags-y += -DCONFIG_SSV_BUILD_AS_ONE_KO
#ccflags-y += -DCONFIG_SSV_SUPPORT_AES_ASM

# Enable -g to help debug. Deassembly from .o to .S would help to track to 
# the problomatic line from call stack dump.
ccflags-y += -g
