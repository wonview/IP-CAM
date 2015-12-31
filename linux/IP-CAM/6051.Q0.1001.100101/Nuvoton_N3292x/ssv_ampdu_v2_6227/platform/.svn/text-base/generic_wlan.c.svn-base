#include <linux/irq.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/regulator/consumer.h>

#if (defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SCX35))
#include <mach/board.h>
#include <mach/pinmap.h>
#endif
#include <mach/hardware.h>
#include <asm/io.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
#include <linux/printk.h>
#include <linux/err.h>
#else
#include <config/printk.h>
#endif
#ifdef CONFIG_SSV_SUPPORT_SP6821A
#include <mach/ldo.h>
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,2,0)
#include <linux/wlan_plat.h>
#else
struct wifi_platform_data {
	int (*set_power)(int val);
	int (*set_reset)(int val);
	int (*set_carddetect)(int val);
	void *(*mem_prealloc)(int section, unsigned long size);
	int (*get_mac_addr)(unsigned char *buf);
	void *(*get_country_code)(char *ccode);
};
#endif

#define GPIO_REG_WRITEL(val, reg)    do{__raw_writel(val, CTL_PIN_BASE + (reg));}while(0)

static int g_wifidev_registered = 0;
static struct semaphore wifi_control_sem;
static struct wifi_platform_data *wifi_control_data = NULL;
static struct resource *wifi_irqres = NULL;
static int g_wifi_irq_rc=0;

/*==Spreadtrum resource begin==================================================*/
#ifdef CONFIG_SSV_SUPPORT_SP6821A
extern int sprd_3rdparty_gpio_wifi_pwd;
extern int sprd_3rdparty_gpio_wifi_reset;
extern int sprd_3rdparty_gpio_wifi_irq;
#define IRQ_RES_NAME "ssv_wlan_irq"
#define WIFI_HOST_WAKE 0xFFFF
#else

    #if (defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SCX35))
        #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,8))
        #define REG_PIN_WIFI_IRQ	REG_PIN_KEYOUT6
        #endif
        static int sprd_3rdparty_gpio_wifi_pwd = GPIO_WIFI_SHUTDOWN;
        static int sprd_3rdparty_gpio_wifi_reset = 0;
        static int sprd_3rdparty_gpio_wifi_irq = GPIO_WIFI_IRQ;
        #define IRQ_RES_NAME "ssv_wlan_irq"
        #define WIFI_HOST_WAKE 0xFFFF
    #else
        #ifdef CONFIG_SSV_SUPPORT_AW_SUNXI
            #define SDIO_ID 1
            #define IRQ_RES_NAME "ssv_wlan_irq"
            #define WIFI_HOST_WAKE 0xFFFF
            extern void sunxi_mci_rescan_card(unsigned id, unsigned insert);
            extern int wifi_pm_gpio_ctrl(char* name, int level);
        #else     
            #define IRQ_RES_NAME "ssv_wlan_irq"
            #define WIFI_HOST_WAKE 0xFFFF
        #endif
        
    #endif
#endif

static int ssv_wifi_power(int on)
{
#if (defined(CONFIG_SSV_SUPPORT_SCX35)||defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
    if(sprd_3rdparty_gpio_wifi_pwd>0)
    {
        if (on)
        {
            printk("sprd_3rdparty_gpio_wifi_pwd[%d]\n",sprd_3rdparty_gpio_wifi_pwd);
            /* Power on trout */
            gpio_free(sprd_3rdparty_gpio_wifi_pwd);
            gpio_request(sprd_3rdparty_gpio_wifi_pwd, "SSV_power");
            gpio_direction_output(sprd_3rdparty_gpio_wifi_pwd, 1);
            printk("SSV_gpio_power_on!\n");
            gpio_free(sprd_3rdparty_gpio_wifi_pwd);
            //LDO_TurnOnLDO(LDO_LDO_WIF0);
            //LDO_SetVoltLevel(LDO_LDO_WIF0,LDO_VOLT_LEVEL1);
        }
        else
        {
            gpio_free(sprd_3rdparty_gpio_wifi_pwd);
            gpio_request(sprd_3rdparty_gpio_wifi_pwd, "SSV_power");
            __gpio_set_value(sprd_3rdparty_gpio_wifi_pwd,0);
            gpio_free(sprd_3rdparty_gpio_wifi_pwd);
            printk("SSV_gpio_power_off!\n");    
        }
    }
    else
    {
        printk("!!sprd_3rdparty_gpio_wifi_pwd[0x%x]\n",sprd_3rdparty_gpio_wifi_pwd);
    }
#endif
#ifdef CONFIG_SSV_SUPPORT_AW_SUNXI
        printk("ssv pwr by esp on=%d\n",on);
	if(on)
	{
		wifi_pm_gpio_ctrl("esp8089_chip_en", 0);
		mdelay(50);
        	wifi_pm_gpio_ctrl("esp8089_chip_en", 1);
	}
	else
	{	
		wifi_pm_gpio_ctrl("esp8089_chip_en", 0);
	}
#endif
    return 0;
}

static int ssv_wifi_reset(int on)
{
#if (defined(CONFIG_SSV_SUPPORT_SCX35)||defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
    if(sprd_3rdparty_gpio_wifi_reset>0)
    {
        if (on)
        {
            printk("sprd_3rdparty_gpio_wifi_reset[0x%x]\n",sprd_3rdparty_gpio_wifi_reset);
            /* Power on trout */
            gpio_request(sprd_3rdparty_gpio_wifi_reset, "SSV_power");
            gpio_direction_output(sprd_3rdparty_gpio_wifi_reset, 1);
            printk("SSV gpio wifi reset on\n");
            gpio_free(sprd_3rdparty_gpio_wifi_reset);
        }
        else
        {
            gpio_request(sprd_3rdparty_gpio_wifi_reset, "SSV_power");
            __gpio_set_value(sprd_3rdparty_gpio_wifi_reset,0);
            gpio_free(sprd_3rdparty_gpio_wifi_reset);
            printk("SSV gpio wifi reset off\n");
        }
    }
    else
    {
        printk("!!sprd_3rdparty_gpio_wifi_reset[0x%x]\n",sprd_3rdparty_gpio_wifi_reset);
    }
#endif
    return 0;
}

int ssv_wifi_set_carddetect(int val)
{
#ifdef CONFIG_SSV_SUPPORT_SP6821A
    printk("SSV:wifi_set_carddetect = %d\n",val);
    if(val){
        LDO_TurnOnLDO(LDO_LDO_SDIO1);
    }else{
        LDO_TurnOffLDO(LDO_LDO_SDIO1);
    }
#endif 

#if 0//def CONFIG_SSV_SUPPORT_SCX35
    if(wlan_mmc) {
        mmc_detect_change(wlan_mmc, 0);
    } else {
        printk(KERN_ERR "%s  wlan_mmc is null,carddetect failed \n ",__func__);
    }
#endif    
#if (defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
        sdhci_bus_scan();    
#endif
    
#ifdef CONFIG_SSV_SUPPORT_AW_SUNXI
        sunxi_mci_rescan_card(SDIO_ID, val);
#endif

    return 0;
}

static struct wifi_platform_data ssv_wifi_control = {
    .set_power = ssv_wifi_power,
    .set_reset = ssv_wifi_reset,
    .set_carddetect = ssv_wifi_set_carddetect,
    //.mem_prealloc   = _mem_prealloc,
};
static struct resource resources[] = {
        {
                .start = WIFI_HOST_WAKE,
                .flags = IORESOURCE_IRQ,
                .name = IRQ_RES_NAME,
        },
};

void ssv_wifi_device_release(struct device *dev)
{
    printk(KERN_INFO "ssv_wifi_device_release\n");
}

static struct platform_device ssv_wifi_device = {
#ifdef CONFIG_SSV_SUPPORT_SCX35
        .name = "xxx_wlan",
#else
        .name = "ssv_wlan",
#endif
        .id = 1,
        .num_resources = ARRAY_SIZE(resources),
        .resource = resources,
        .dev = {
                .platform_data = &ssv_wifi_control,
                .release = ssv_wifi_device_release,
         },
};

/*==Spreadtrum SP6821A resource end==================================================*/

/*==Rockchip RK30 resource begin==================================================*/
#ifdef CONFIG_SUPPORT_ROCKCHIP
#define IRQ_RES_NAME "bcmdhd_wlan_irq"
#define WIFI_HOST_WAKE RK30_PIN3_PD2

#endif
/*==Rockchip RK30 resource end==================================================*/

int wifi_set_power(int on, unsigned long msec)
{
	if (wifi_control_data && wifi_control_data->set_power) {
		wifi_control_data->set_power(on);
	}
	if (msec)
		msleep(msec);
	return 0;
}

int wifi_set_reset(int on, unsigned long msec)
{
	if (wifi_control_data && wifi_control_data->set_reset) {
		wifi_control_data->set_reset(on);
	}
	if (msec)
		msleep(msec);
	return 0;
}

static int wifi_set_carddetect(int on)
{
	if (wifi_control_data && wifi_control_data->set_carddetect) {
		wifi_control_data->set_carddetect(on);
	}
	return 0;
}

static irqreturn_t wifi_wakeup_irq_handler(int irq, void *dev){
    printk("sdhci_wakeup_irq_handler\n");
    /* Disable interrupt before calling handler */
     disable_irq_nosync(irq);

         return IRQ_HANDLED;

}

void setup_wifi_wakeup_BB(struct platform_device *pdev, bool bEnable)
{
    int rc=0,ret=0;
    if (bEnable){
	    wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, IRQ_RES_NAME);
#if (defined(CONFIG_SSV_SUPPORT_SCX35)||defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
    
#ifdef CONFIG_SSV_SUPPORT_SP8825EA
        {
            unsigned long gpio_cfg = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_SLP_IE);    
            GPIO_REG_WRITEL(gpio_cfg, REG_PIN_WIFI_IRQ);
        }
#endif
        if(sprd_3rdparty_gpio_wifi_irq>0)
        {
            gpio_free(sprd_3rdparty_gpio_wifi_irq);

            //sprd_mfp_config(&gpio_cfg, 1);
            rc =gpio_request(sprd_3rdparty_gpio_wifi_irq, "SSV int");
            if(rc){
                printk(KERN_ALERT "cannot request gpio for wifi irq\n");
                //goto err_out0;
            }

            gpio_direction_input(sprd_3rdparty_gpio_wifi_irq);
#ifdef CONFIG_SSV_SUPPORT_SP6821A
            rc = sprd_alloc_gpio_irq(sprd_3rdparty_gpio_wifi_irq);
#endif
#ifdef CONFIG_SSV_SUPPORT_SP8825EA
            rc = __gpio_to_irq(sprd_3rdparty_gpio_wifi_irq);
#endif
            if(rc < 0){
                printk(KERN_ALERT "cannot alloc irq for wifi irq gpio\n");
                gpio_free(sprd_3rdparty_gpio_wifi_irq);
                //goto err_out0;
            }
        }
        else
        {
            printk("trout:sprd_3rdparty_gpio_wifi_irq[0x%x]\n",sprd_3rdparty_gpio_wifi_irq);
            //goto err_out0;
        }
        wifi_irqres->start = (resource_size_t)rc;
#else
        rc = (int)wifi_irqres->start;
#endif
        g_wifi_irq_rc = rc;
        ret = request_threaded_irq(rc,
                                    NULL,
                                    (void *)wifi_wakeup_irq_handler,
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
                                    IRQ_TYPE_LEVEL_HIGH | IRQF_ONESHOT |IRQF_FORCE_RESUME,
#else
                                    IRQ_TYPE_LEVEL_HIGH | IRQF_ONESHOT,
#endif
                                    "wlan_wakeup_irq", NULL);
        enable_irq_wake(g_wifi_irq_rc);
    }else{
        if(g_wifi_irq_rc){
            free_irq(g_wifi_irq_rc,NULL);
            g_wifi_irq_rc = 0;
        }
        
#if (defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
        if(sprd_3rdparty_gpio_wifi_irq>0)
        {
#ifdef CONFIG_SSV_SUPPORT_SP6821A
            sprd_free_gpio_irq(g_wifi_irq_rc);
#endif
            gpio_free(sprd_3rdparty_gpio_wifi_irq);
        }
#endif
    }
    //printk(KERN_INFO "%s=%d,sprd_3rdparty_gpio_wifi_irq=%d\n",__FUNCTION__,bEnable,sprd_3rdparty_gpio_wifi_irq);        
}
static int wifi_probe(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl =
		(struct wifi_platform_data *)(pdev->dev.platform_data);

	printk(KERN_ALERT "wifi_probe\n");

    //setup_wifi_wakeup_BB(pdev,true);

    wifi_control_data = wifi_ctrl;
#if (defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
    sdhci_device_attach(1);
#endif
    wifi_set_power(0,40);
    wifi_set_power(1,10);
    //wifi_set_reset(1, 12);	
    wifi_set_carddetect(1);

    up(&wifi_control_sem);
    return 0;
}

static int wifi_remove(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl =
		(struct wifi_platform_data *)(pdev->dev.platform_data);

	wifi_control_data = wifi_ctrl;

	wifi_set_power(0, 0);	/* Power Off */
	//wifi_set_reset(0, 0);
	
	wifi_set_carddetect(0);	/* CardDetect (1->0) */
#if (defined(CONFIG_SSV_SUPPORT_SP8825EA)||defined(CONFIG_SSV_SUPPORT_SP6821A))
    sdhci_device_attach(0);
#endif
    
    setup_wifi_wakeup_BB(pdev,false);

	return 0;
}

static int wifi_suspend(struct platform_device *pdev, pm_message_t state)
{
    //printk(KERN_INFO "##> %s,g_wifi_irq_rc=%d\n", __FUNCTION__,g_wifi_irq_rc);	
    setup_wifi_wakeup_BB(pdev,true);
    return 0;
}

static int wifi_resume(struct platform_device *pdev)
{
    //printk(KERN_INFO "##> %s,g_wifi_irq_rc=%d\n", __FUNCTION__,g_wifi_irq_rc);
    setup_wifi_wakeup_BB(pdev,false);    
    return 0;
}

static struct platform_driver wifi_driver = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = wifi_suspend,
	.resume         = wifi_resume,
	.driver         = {
#ifdef CONFIG_SUPPORT_ROCKCHIP
	.name   = "bcmdhd_wlan",
#endif    
	.name   = "ssv_wlan",
	}
};

extern int ssvdevice_init(void);
extern void ssvdevice_exit(void);
#ifdef CONFIG_SSV_SUPPORT_AES_ASM
extern int aes_init(void);
extern void aes_fini(void);
extern int sha1_mod_init(void);
extern void sha1_mod_fini(void);
#endif
int initWlan(void)
{
    int ret=0;
    sema_init(&wifi_control_sem, 0);
#ifdef CONFIG_SSV_SUPPORT_AES_ASM
    sha1_mod_init();
    aes_init();
#endif

#ifndef CONFIG_SUPPORT_ROCKCHIP	
    platform_device_register(&ssv_wifi_device);
#endif	

    platform_driver_register(&wifi_driver);

    g_wifidev_registered = 1;

    /* Waiting callback after platform_driver_register is done or exit with error */
    if (down_timeout(&wifi_control_sem,  msecs_to_jiffies(1000)) != 0) {
        ret = -EINVAL;
        printk(KERN_ALERT "%s: platform_driver_register timeout\n", __FUNCTION__);
    }
    ret = ssvdevice_init();
    return ret;
}

void exitWlan(void)
{
    if (g_wifidev_registered)
    {
        ssvdevice_exit();
#ifdef CONFIG_SSV_SUPPORT_AES_ASM
        aes_fini();
        sha1_mod_fini();
#endif
        platform_driver_unregister(&wifi_driver);
#ifndef CONFIG_SUPPORT_ROCKCHIP	
        platform_device_unregister(&ssv_wifi_device);
#endif
        g_wifidev_registered = 0;
    }
    return;
}
#ifdef CONFIG_SUPPORT_ROCKCHIP
int rockchip_wifi_init_module(void)
#else
static int generic_wifi_init_module(void)
#endif
{
	return initWlan();
}

#ifdef CONFIG_SUPPORT_ROCKCHIP
void rockchip_wifi_exit_module(void)
#else
static void generic_wifi_exit_module(void)
#endif
{
	exitWlan();
}
#ifdef CONFIG_SUPPORT_ROCKCHIP
EXPORT_SYMBOL(rockchip_wifi_init_module);
EXPORT_SYMBOL(rockchip_wifi_exit_module);
#else
EXPORT_SYMBOL(generic_wifi_init_module);
EXPORT_SYMBOL(generic_wifi_exit_module);
module_init(generic_wifi_init_module);
module_exit(generic_wifi_exit_module);
#endif
MODULE_LICENSE("Dual BSD/GPL");
