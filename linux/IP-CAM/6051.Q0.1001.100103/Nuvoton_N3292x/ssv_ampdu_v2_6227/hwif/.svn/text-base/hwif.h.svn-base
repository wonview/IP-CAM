/*
 * Copyright (c) 2008 Atheros Communications Inc.
 * Copyright (c) 2009 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (c) 2009 Imre Kaloz <kaloz@openwrt.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _LINUX_SSVCABRIO_PLATFORM_H
#define _LINUX_SSVCABRIO_PLATFORM_H


#include <linux/mmc/host.h>
#include <hwif/sdio/sdio_def.h>




#define SSVCABRIO_PLAT_EEP_MAX_WORDS	2048



/**
* Macros for ssv6200 register read/write access on Linux platform.
* @ SSV_REG_WRITE() : write 4-byte value into hardware register.
* @ SSV_REG_READ()  : read 4-byte value from hardware register.
*             
*/
#define SSV_REG_WRITE(dev, reg, val) \
        (sh)->priv->ops->writereg((sh)->sc->dev, (reg), (val))
#define SSV_REG_READ(dev, reg, buf)  \
        (sh)->priv->ops->readreg((sh)->sc->dev, (reg), (buf))


#if 0

/**
* Macros for ssv6200 register read/write access on Linux platform.
* @ SSV_REG_WRITE() : write 4-byte value into hardware register.
* @ SSV_REG_READ()  : read 4-byte value from hardware register.
* @ SSV_REG_SET_BITS: set the specified bits to a value.
*             
*/
#define SSV_REG_WRITE(sh, reg, val) \
        (sh)->priv->ops->writereg((sh)->sc->dev, (reg), (val))
#define SSV_REG_READ(sh, reg, buf)  \
        (sh)->priv->ops->readreg((sh)->sc->dev, (reg), (buf))
#define SSV_REG_CONFIRM(sh, reg, val)       \
{                                           \
    u32 regval;                             \
    SSV_REG_READ(sh, reg, &regval);         \
    if (regval != (val)) {                  \
        printk("[0x%08x]: 0x%08x!=0x%08x\n",\
        (reg), (val), regval);              \
        return -1;                          \
    }                                       \
}

#define SSV_REG_SET_BITS(sh, reg, set, clr) \
{                                           \
    u32 reg_val;                            \
    SSV_REG_READ(sh, reg, &reg_val);        \
    reg_val &= ~(clr);                      \
    reg_val |= (set);                       \
    SSV_REG_WRITE(sh, reg, reg_val);        \
}
#endif


/**
* Hardware Interface (SDIP/SPI) APIs for ssv6200 on Linux platform.
*/
struct ssv6xxx_hwif_ops {
    int __must_check (*read)(struct device *child, void *buf,size_t *size);
    int __must_check (*write)(struct device *child, void *buf, size_t len,u8 queue_num);
    int __must_check (*readreg)(struct device *child, u32 addr, u32 *buf);
    int __must_check (*writereg)(struct device *child, u32 addr, u32 buf);
    int (*irq_getmask)(struct device *child, u32 *mask);
    void (*irq_setmask)(struct device *child,int mask);
    void (*irq_enable)(struct device *child);
    void (*irq_disable)(struct device *child,bool iswaitirq);
    int (*irq_getstatus)(struct device *child,int *status);
    void (*irq_request)(struct device *child,irq_handler_t irq_handler,void *irq_dev);
    void (*irq_trigger)(struct device *child);
	void (*pmu_wakeup)(struct device *child);
    int __must_check (*load_fw)(struct device *child);
    int (*cmd52_read)(struct device *child, u32 addr, u32 *value);
    int (*cmd52_write)(struct device *child, u32 addr, u32 value);
    bool (*support_scatter)(struct device *child);    
    int (*rw_scatter)(struct device *child, struct sdio_scatter_req *scat_req);    
};







/**
*
*/
struct ssv6xxx_if_debug {
    struct device *dev;
    struct platform_device *pdev;

};

struct ssvcabrio_platform_data {
    //use to avoid remove mmc cause dead lock.
    atomic_t            irq_handling;    
    bool                is_enabled;
	unsigned short		vendor;		/* vendor id */
	unsigned short		device;		/* device id */
	struct ssv6xxx_hwif_ops *ops;
};

#endif /* _LINUX_SSVCABRIO_PLATFORM_H */
