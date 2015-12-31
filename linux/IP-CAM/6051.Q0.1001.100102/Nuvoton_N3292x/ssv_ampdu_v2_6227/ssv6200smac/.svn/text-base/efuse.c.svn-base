/*
    4bits as efuse access unit.
*/
#include <linux/etherdevice.h>
#include <ssv6200.h>
#include "efuse.h"

#ifdef CONFIG_SSV_EXTERNAL_MAC
extern char ssvcabrio_wifimac[50];
#define EXTERNAL_MAC_INPUT_PATH ssvcabrio_wifimac

mm_segment_t oldfs;

struct file *openFile(char *path,int flag,int mode)
{
    struct file *fp=NULL;

    fp=filp_open(path, flag, 0);
    if(IS_ERR(fp))
        return NULL;
    else
        return fp;
}

int readFile(struct file *fp,char *buf,int readlen)
{
    if (fp->f_op && fp->f_op->read)
        return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
    else
    return -1;
}

int closeFile(struct file *fp)
{
    filp_close(fp,NULL);
    return 0;
}

void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

void parseMac(char* mac, u_int8_t addr[])
{
    long b;
    int i;
    for (i = 0; i < 6; i++)
    {
        b = simple_strtol(mac+(3*i), (char **) NULL, 16);
        addr[i] = (char)b;
    }
}

static int readfile_mac(u8 *mac_addr) 
{
    char buf[128];
    struct file *fp=NULL;
    int ret=0;

    fp=openFile(EXTERNAL_MAC_INPUT_PATH,O_RDONLY,0);
    if (fp!=NULL)
    {
        initKernelEnv();
        memset(buf,0,128); 
        if ((ret=readFile(fp,buf,128))>0)
        {
            parseMac(buf,(uint8_t *)mac_addr);
        }
        else
            printk("read file error %d\n",ret);
        set_fs(oldfs);
        closeFile(fp);
    }
    return ret;
}
#endif // CONFIG_SSV_EXTERNAL_MAC

#define EFUSE_HWSET_MAX_SIZE 256
#define EFUSE_MAX_SECTION_MAP (EFUSE_HWSET_MAX_SIZE>>3)

#define SSV_EFUSE_BASE	0xC2000108

static struct efuse_map SSV_EFUSE_ITEM_TABLE[] = {
    {4, 0, 0},
    {4, 8, 0},
    {4, 8, 0},
    {4, 48, 0},//Mac address
    {4, 8, 0},
    {4, 8, 0},
    {4, 12, 0},//EFUSE_IQ_CALIBRAION_RESULT
    {4, 8, 0},
    {4, 8, 0},
};

static u8 read_efuse(struct ssv_hw *sh, u8 *pbuf)
{
    u32 *pointer32, i;

    pointer32 = (u32 *)pbuf;
    SMAC_REG_READ(sh,SSV_EFUSE_BASE,pointer32);
    if (*pointer32 == 0x00) {
		return 0;
    }

    for (i=0; i<EFUSE_MAX_SECTION_MAP; i+=4, pointer32++)
        SMAC_REG_READ(sh,SSV_EFUSE_BASE+i,pointer32);

    return 1;
}

static u16 parser_efuse(u8 *pbuf, u8 *mac_addr)
{
    u8 *rtemp8,idx=0;
	u16 shift=0,i;
    //bit as unit
    u16 efuse_real_content_len = 0;

	rtemp8 = pbuf;

    if (*rtemp8 == 0x00) {
		return efuse_real_content_len;
    }

	do
	{
		idx = (*(rtemp8) >> shift)&0xf;
		switch(idx)
		{
            //1 byte type
			case EFUSE_R_CALIBRAION_RESULT:
			case EFUSE_CRYSTAL_FREQUECY_OFFSET:
			case EFUSE_DC_CALIBRAION_RESULT:
			case EFUSE_TX_POWER_INDEX_1:
			case EFUSE_TX_POWER_INDEX_2:
			case EFUSE_SAR_RESULT:
				if(shift)
				{
					rtemp8 ++;
					SSV_EFUSE_ITEM_TABLE[idx].value = (u16)((u8)(*((u16*)rtemp8)) & ((1<< SSV_EFUSE_ITEM_TABLE[idx].byte_cnts) - 1));
				}
				else
				{
					SSV_EFUSE_ITEM_TABLE[idx].value = (u16)((u8)(*((u16*)rtemp8) >> 4) & ((1<< SSV_EFUSE_ITEM_TABLE[idx].byte_cnts) - 1));
				}
				efuse_real_content_len += (SSV_EFUSE_ITEM_TABLE[idx].offset + SSV_EFUSE_ITEM_TABLE[idx].byte_cnts);
				break;
            //MAC address
			case EFUSE_MAC:
                if(shift)
				{
					rtemp8 ++;
					memcpy(mac_addr,rtemp8,6);
				}
				else
				{
					for(i=0;i<6;i++)
					{
						mac_addr[i] = (u16)(*((u16*)rtemp8) >> 4) & 0xff;
						rtemp8++;
					}
				}
				efuse_real_content_len += (SSV_EFUSE_ITEM_TABLE[idx].offset + SSV_EFUSE_ITEM_TABLE[idx].byte_cnts);
				break;
            //2 bytes type
			case EFUSE_IQ_CALIBRAION_RESULT:
				if(shift)
				{
					rtemp8 ++;
					SSV_EFUSE_ITEM_TABLE[idx].value = (u16)(*((u16*)rtemp8)) & ((1<< SSV_EFUSE_ITEM_TABLE[idx].byte_cnts) - 1);
				}
				else
				{
					SSV_EFUSE_ITEM_TABLE[idx].value = (u16)(*((u16*)rtemp8) >> 4) & ((1<< SSV_EFUSE_ITEM_TABLE[idx].byte_cnts) - 1);
				}
				efuse_real_content_len += (SSV_EFUSE_ITEM_TABLE[idx].offset + SSV_EFUSE_ITEM_TABLE[idx].byte_cnts);
				break;
			default:
                idx = 0;
				break;
		}
		shift = efuse_real_content_len % 8;
		rtemp8 = &pbuf[efuse_real_content_len / 8];
	}while(idx != 0);
    return efuse_real_content_len;
}

void addr_increase_copy(u8 *dst, u8 *src)
{
#if 0
	u16 *a = (u16 *)dst;
	const u16 *b = (const u16 *)src;

	//xx:xx:xx:xx:ff:00 -> xx:xx:xx:xx:00:01
	//xx:xx:xx:xx:ff:ff -> xx:xx:xx:xx:fe:ff
	a[0] = b[0];
	a[1] = b[1];
	if (b[2] == 0xffff)
		a[2] = b[2] - 1;
	else
		a[2] = b[2] + 1;
#endif
    u8 *a = (u8 *)dst;
    const u8 *b = (const u8 *)src;

    a[0] = b[0];
    a[1] = b[1];
    a[2] = b[2];
    a[3] = b[3];
    a[4] = b[4];
    if (b[5] == 0xff)
        a[5] = b[5] - 1;
    else
        a[5] = b[5] + 1;

}

//#define EFUSE_DEBUG
void efuse_read_all_map(struct ssv_hw *sh)
{
    extern struct ssv6xxx_cfg ssv_cfg;
    u8 efuse_mapping_table[EFUSE_MAX_SECTION_MAP+1];
#ifndef CONFIG_SSV_RANDOM_MAC
    u8 pseudo_mac0[ETH_ALEN] = { 0x20, 0x33, 0x33, 0x33, 0x33, 0x33 };
#endif
    u8 rom_mac0[ETH_ALEN];
#ifdef EFUSE_DEBUG
    int i;
#endif
    memset(rom_mac0,0x00,ETH_ALEN);
	memset(efuse_mapping_table,0x00,EFUSE_MAX_SECTION_MAP+1);
    read_efuse(sh, efuse_mapping_table);

#ifdef EFUSE_DEBUG
    for(i=0;i<EFUSE_MAX_SECTION_MAP;i++)
    {
        if(i%16 == 0)
            printk("\n");
        printk("%02x-",efuse_mapping_table[i]);
    }
    printk("\n");
#endif

    //Parser data. 
    parser_efuse(efuse_mapping_table,rom_mac0);
    
    //1 byte type
    ssv_cfg.r_calbration_result = (u8)SSV_EFUSE_ITEM_TABLE[EFUSE_R_CALIBRAION_RESULT].value;
    ssv_cfg.sar_result = (u8)SSV_EFUSE_ITEM_TABLE[EFUSE_SAR_RESULT].value;
    ssv_cfg.crystal_frequecy_offse = (u8)SSV_EFUSE_ITEM_TABLE[EFUSE_CRYSTAL_FREQUECY_OFFSET].value;
    ssv_cfg.dc_calbration_result = (u8)SSV_EFUSE_ITEM_TABLE[EFUSE_DC_CALIBRAION_RESULT].value;
    ssv_cfg.tx_power_index_1 = (u8)SSV_EFUSE_ITEM_TABLE[EFUSE_TX_POWER_INDEX_1].value;
    ssv_cfg.tx_power_index_2 = (u8)SSV_EFUSE_ITEM_TABLE[EFUSE_TX_POWER_INDEX_2].value;
    //2 bytes type
    ssv_cfg.iq_calbration_result = (u16)SSV_EFUSE_ITEM_TABLE[EFUSE_IQ_CALIBRAION_RESULT].value;

    //MAC address from insmode ... 
    if (!is_valid_ether_addr(&sh->cfg.maddr[0][0]))
    {
#ifdef CONFIG_SSV_EXTERNAL_MAC
        //MAC address from /data/wifimac ...
        if((readfile_mac(&sh->cfg.maddr[0][0])) && (is_valid_ether_addr(&sh->cfg.maddr[0][0])))
        {
            addr_increase_copy(&sh->cfg.maddr[1][0], &sh->cfg.maddr[0][0]);
        }
        else//MAC address from E-fuse ...
#endif // CONFIG_SSV_EXTERNAL_MAC
        {
            //If ROM address is invalid. use pseudo mac address.
            if (is_valid_ether_addr(rom_mac0)) {
			    memcpy(&sh->cfg.maddr[0][0], rom_mac0, ETH_ALEN);
			    addr_increase_copy(&sh->cfg.maddr[1][0], rom_mac0);
            }
		    else
		    {
#ifdef CONFIG_SSV_RANDOM_MAC
                get_random_bytes(&sh->cfg.maddr[0][0],ETH_ALEN);
                //Avoid Broadcast address & Multicast address
                sh->cfg.maddr[0][0] = sh->cfg.maddr[0][0] & 0xF0;
                addr_increase_copy(&sh->cfg.maddr[1][0], &sh->cfg.maddr[0][0]);
#else
                memcpy(&sh->cfg.maddr[0][0], pseudo_mac0, ETH_ALEN);
                addr_increase_copy(&sh->cfg.maddr[1][0], pseudo_mac0);
#endif
            }
        }
    }
		

#ifdef EFUSE_DEBUG
    printk("@@ - EFUSE configuration\n");
    printk("@@ r_calbration_result- %x\n",ssv_cfg.r_calbration_result);
    printk("@@ sar_result- %x\n",ssv_cfg.sar_result);
    printk("@@ crystal_frequecy_offse- %x\n",ssv_cfg.crystal_frequecy_offse);
    printk("@@ dc_calbration_result- %x\n",ssv_cfg.dc_calbration_result);
    printk("@@ iq_calbration_result- %x\n",ssv_cfg.iq_calbration_result);
    printk("@@ tx_power_index_1- %x\n",ssv_cfg.tx_power_index_1);
    printk("@@ tx_power_index_2- %x\n",ssv_cfg.tx_power_index_2);
	printk("@@ rom_mac0- %pM\n", rom_mac0);
#endif
}
 
