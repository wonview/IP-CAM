/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This hardware has more or less no CMD53 support, so all registers
 * must be accessed using sdio_readb()/sdio_writeb().
 *
 * Transfers must be in one transaction or the firmware goes bonkers.
 * This means that the transfer must either be small enough to do a
 * byte based transfer or it must be padded to a multiple of the
 * current block size.
 *
 * As SDIO is still new to the kernel, it is unfortunately common with
 * bugs in the host controllers related to that. One such bug is that
 * controllers cannot do transfers that aren't a multiple of 4 bytes.
 * If you don't have time to fix the host controller driver, you can
 * work around the problem by modifying ssv_sdio_host_to_card() and
 * ssv_sdio_card_to_host() to pad the data.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/pm_runtime.h>

#include "host.h"
#include "decl.h"
#include "defs.h"
#include "dev.h"
#include "cmd.h"
#include "ssv_sdio.h"
#include "../include/cabrio.h"
#include "../include/ssv6200_reg.h"
#include "cabrio-if/cabrio_host_types.h"
#include "cabrio-if/ssv_pktdef.h"
#include "cabrio-if/cabrio_cfg.h"

#define RX_IN_WORKER
#define CHECK_HCI_INQ_FULL

#define MAX_QUEUED_PACKET         (64)
#define MAX_TX_PACKET_COUNT       (8)

static void cabrio_sdio_interrupt(struct sdio_func *func);

// #define DEBUG_SDIO_CLAIMER

//#define ENABLE_TX_DONE_INT
#define CABRIO_SDIO_BLOCK_SIZE      SDIO_DEF_BLOCK_SIZE
/* The if_sdio_remove() callback function is called when
 * user removes this module from kernel space or ejects
 * the card from the slot. The driver handles these 2 cases
 * differently for SD8688 combo chip.
 * If the user is removing the module, the FUNC_SHUTDOWN
 * command for SD8688 is sent to the firmware.
 * If the card is removed, there is no need to send this command.
 *
 * The variable 'user_rmmod' is used to distinguish these two
 * scenarios. This flag is initialized as FALSE in case the card
 * is removed, and will be set to TRUE for module removal when
 * module_exit function is called.
 */
static u8 user_rmmod;

static char *cabrio_fw_name = NULL;
module_param_named(fw_name, cabrio_fw_name, charp, 0644);

static const struct sdio_device_id cabrio_sdio_ids[] = {
    { SDIO_DEVICE(SSV_VENDOR_ID, SSV_CABRIO_DEVID) },
    { /* end: all zeroes */                },
};

MODULE_DEVICE_TABLE(sdio, cabrio_sdio_ids);

#ifdef DEBUG_SDIO_CLAIMER
static int sdio_claimer        = 0;

#define SDIO_CLAIM_HOST(func)        \
    do { \
        pr_err("Host claimed by %d, %d, %08X.\n", sdio_claimer, func->card->host->claimed, (u32)func->card->host->claimer); \
        sdio_claim_host(func); \
        sdio_claimer = __LINE__; \
        pr_err("Claim: %d, %08X, %d\n", sdio_claimer, current_thread_info()->task, (u32)func->card->host->claim_cnt); \
    } while (0)

#define SDIO_RELEASE_HOST(func)        \
    do { \
        sdio_release_host(func); \
        pr_err("Host released @%d by %d, %d, %08X, %d.\n", __LINE__, sdio_claimer, func->card->host->claimed, (u32)func->card->host->claimer, (u32)func->card->host->claim_cnt); \
        sdio_claimer = 0; \
    } while (0)
#else // DEBUG_SDIO_CLAIMER
#define SDIO_CLAIM_HOST(func)            sdio_claim_host(func)

#define SDIO_RELEASE_HOST(func)          sdio_release_host(func)

#endif // DEBUG_SDIO_CLAIMER

#define MODEL_6200    0x01

static const struct cabrio_fw_table fw_table[] = {
    { MODEL_6200, "ssv6200-uart.bin"},
    { 0, NULL}
};

MODULE_FIRMWARE("ssv6200-uart.bin");

struct cabrio_sdio_packet {
    struct cabrio_sdio_packet    *next;
    u16                           nb;
    u8                            buffer[0] __attribute__((aligned(4)));
};

struct cabrio_sdio_card {
    struct sdio_func             *func;
    struct cabrio_private        *priv;

    u32                           reg_io_addr;
    u32                           data_io_addr;

    int                           model;
    u32                           block_size;

    unsigned long                 ioport;
    unsigned int                  scratch_reg;

    const char                   *firmware;
    bool                          firmware_allocated;

    u8                            buffer[65536] __attribute__((aligned(4)));

    spinlock_t                    lock;
    bool                          in_worker;
    struct cabrio_sdio_packet    *packets;
    u32                           packet_num;

    struct workqueue_struct      *workqueue;
    struct work_struct            packet_worker;
    bool                          en_worker;

    u8                            rx_unit;
};

/*******************************************************************/
/* Cabrio access functions                                         */
/*******************************************************************/
// Initialize Cabrio SDIO access
int cabrio_sdio_init(struct cabrio_sdio_card *card)
{
    int               err_ret = 0;
    struct sdio_func *func    = card->func;
    u32               data_io_addr, reg_io_addr;

    SDIO_CLAIM_HOST(func);
    
    do {
    sdio_set_max_clock(func, 25000000);
    //sdio_set_max_clock(func, 12500000);
    //sdio_set_max_clock(func, 6250000);

    /* Grab access to FN0 for ELP reg. */
    func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

    /* Use block mode for transferring over one block size of data */
    func->card->quirks |= MMC_QUIRK_BLKSZ_FOR_BYTE_MODE;

    //get dataIOPort
    data_io_addr = (u32)sdio_readb(func, 0x00, &err_ret);
    if (err_ret) break;
    data_io_addr = data_io_addr | (sdio_readb(func, 0x01, &err_ret) << 8);
    if (err_ret) break;
    data_io_addr = data_io_addr | (sdio_readb(func, 0x02, &err_ret) << 16);
    if (err_ret) break;
    card->data_io_addr = data_io_addr;

    //get regIOPort
    reg_io_addr = sdio_readb(func, 0x70, &err_ret);
    if (err_ret) break;
    reg_io_addr = reg_io_addr | (sdio_readb(func, 0x71, &err_ret) << 8);
    if (err_ret) break;
    reg_io_addr = reg_io_addr | (sdio_readb(func, 0x72, &err_ret) << 16);
    if (err_ret) break;
    card->reg_io_addr = reg_io_addr;
        
    cabrio_dbg_sdio("data_io_addr 0x%x, reg_io_addr 0x%x\n",
                    card->data_io_addr, card->reg_io_addr);

    if (card->block_size != 0) {
        sdio_set_block_size(func, card->block_size);
        cabrio_dbg_sdio("Using block size %d\n", card->block_size);
    }

    // Enable TX allocation 
    sdio_writeb(func, (SDIO_TX_ALLOC_ENABLE | SDIO_TX_ALLOC_SIZE_SHIFT), REG_SDIO_TX_ALLOC_SHIFT, &err_ret);
    if (err_ret) break;
    
    // mask rx/tx complete int
    // 0: rx int
    // 1: tx complete int
    #if !defined(CONFIG_RX_POLL) || (CONFIG_RX_POLL == 0)
    #ifdef ENABLE_TX_DONE_INT
    sdio_writeb(func, 0x08, REG_INT_MASK, &err_ret);
    #else
    sdio_writeb(func, 0x08|0x02, REG_INT_MASK, &err_ret);
    #endif // ENABLE_TX_DONE_INT
    #else
    sdio_writeb(func, 0x0B, REG_INT_MASK, &err_ret);
    #endif

    // output timing
    sdio_writeb(func, SDIO_DEF_OUTPUT_TIMING, 0x55, &err_ret);
    if (err_ret) break;
    
    // switch to normal mode
    // bit[1] , 0:normal mode, 1: Download mode
    sdio_writeb(func, 0x00, 0x0c, &err_ret);
    } while (0);

    SDIO_RELEASE_HOST(func);

    return err_ret;
} // end of - cabrio_sdio_init -


// Write one word to CABRIO address space
int cabrio_sdio_write_reg(struct cabrio_sdio_card *card, u32 addr, u32 data)
{
    int ret = 0;
    struct sdio_func *func = card->func;
    u8 sdio_data[8];

    //8 byte ( 4 bytes address , 4 bytes data )
    // 4 bytes address
    sdio_data[0] = (u8)addr;
    sdio_data[1] = (u8)(addr >> 8);
    sdio_data[2] = (u8)(addr >> 16);
    sdio_data[3] = (u8)(addr >> 24);

    // 4 bytes data    
    sdio_data[4] = (u8)data;
    sdio_data[5] = (u8)(data >> 8);
    sdio_data[6] = (u8)(data >> 16);
    sdio_data[7] = (u8)(data >> 24);

    SDIO_CLAIM_HOST(func);
    ret = sdio_memcpy_toio(func, card->reg_io_addr, sdio_data, 8);
    SDIO_RELEASE_HOST(func);

    if (WARN_ON(ret))
        cabrio_dbg_sdio("Cabrio reg write failed (%d)\n", ret);

    cabrio_dbg_sdio("Write reg 0x%x: 0x%x (ret:%d)\n",
                    addr, data, ret);

    return ret;
} // end of - cabrio_sdio_write_reg

    
// Read one workd from CABRIO address space
int cabrio_sdio_read_reg(struct cabrio_sdio_card *card, u32 addr, u32 *p_data)
{
    int ret = 0;
    struct sdio_func *func = card->func;
    u8 sdio_data[4];

    // 4 bytes address
    sdio_data[0] = (u8)addr;
    sdio_data[1] = (u8)(addr >> 8);
    sdio_data[2] = (u8)(addr >> 16);
    sdio_data[3] = (u8)(addr >> 24);

    SDIO_CLAIM_HOST(func);
    // Write address
    ret = sdio_memcpy_toio(func, card->reg_io_addr, sdio_data, 4);

    if (WARN_ON(ret))
        cabrio_dbg_sdio("Cabrio reg write failed (%d)\n", ret);

    // Read data
    ret = sdio_memcpy_fromio(func, sdio_data, card->reg_io_addr, 4);

    SDIO_RELEASE_HOST(func);

    if (WARN_ON(ret))
        cabrio_dbg_sdio("Cabrio reg read failed (%d)\n", ret);

    *p_data = (u32)sdio_data[0] + ((u32)sdio_data[1] << 8) + 
              ((u32)sdio_data[2] << 16) + ((u32)sdio_data[3] << 24); 

    cabrio_dbg_sdio("Read 0x%x: 0x%x (ret:%d)\n",
                    addr, *p_data, ret);
    return ret;
}

static int _cabrio_sdio_read_data (struct cabrio_sdio_card *card,
                                   void *buf, size_t len)
{
    int               ret = 0;
    struct sdio_func *func = card->func;
    size_t            readsize = sdio_align_size(func, len);

    cabrio_dbg_sdio("sdio read 53, %zu/%zu bytes\n", len, readsize);

    SDIO_CLAIM_HOST(func);

    ret = sdio_memcpy_fromio(func, buf, card->data_io_addr, readsize);

    SDIO_RELEASE_HOST(func);

    if (ret)
        pr_err("Data read failed (%d)\n", ret);

   return ret;
}

#if 0
static int _cbrio_sdio_get_irq(struct cabrio_sdio_card *card, int *status)
{
    int err_ret;
    struct sdio_func *func = card->func;

    SDIO_CLAIM_HOST(func);
    *status = sdio_readb(func, 0x08, &err_ret);
    SDIO_RELEASE_HOST(func);

    cabrio_dbg_sdio("irq: %08X\n", *status);
    return err_ret;
}
#endif // 0

#ifdef CHECK_HCI_INQ_FULL
static int is_hci_iq_full (struct cabrio_sdio_card *card)
{
    u32        reg_data;
    cabrio_sdio_read_reg(card, 0xcd00001c, &reg_data);

    return ((reg_data & 0x0c) == 2);
}
#endif //  CHECK_HCI_INQ_FULL

static int cabrio_sdio_card_to_host(struct cabrio_sdio_card *card);

static s32 cabrio_rx (struct cabrio_sdio_card *card)
{
    int ret;
    s32 received = 0;
    u8 cause;

    //cabrio_dbg_enter(CABRIO_DBG_SDIO);

    #ifdef CHECK_HCI_INQ_FULL
    if (is_hci_iq_full(card))
        return 0;
    #endif //  CHECK_HCI_INQ_FULL

    // Get INT status
    SDIO_CLAIM_HOST(card->func);
    cause = sdio_readb(card->func, REG_INT_STATUS, &ret);
    sdio_writeb(card->func, ~cause, REG_INT_STATUS, &ret);
    SDIO_RELEASE_HOST(card->func);
    // Filter out those interested.
    #ifdef ENABLE_TX_DONE_INT
    cause &= (SDIO_RX_READY_INT | SDIO_TX_DONE_INT);
    #else
    cause &= (SDIO_RX_READY_INT);
    #endif
    if (ret || (cause  == 0))
        goto out;

    //cabrio_dbg_sdio("interrupt: 0x%X\n", (unsigned)cause);
#if 0
    SDIO_CLAIM_HOST(card->func);
    sdio_writeb(card->func, ~cause, REG_INT_STATUS, &ret);
    SDIO_RELEASE_HOST(card->func);
    if (ret)
        goto out;
#endif
    /*
     * Ignore the define name, this really means the card has
     * successfully received the command.
     */
    card->priv->is_activity_detected = 1;
    // Read packet if RX is ready
    if (cause & SDIO_RX_READY_INT) {
        ret = cabrio_sdio_card_to_host(card);
        if (ret)
            goto out;
        received = 1;
    }
    // Clear TX done and notify TX done.
    #ifdef ENABLE_TX_DONE_INT
    if (cause & SDIO_TX_DONE_INT) {
        // Clear TX done interrupt.
        SDIO_CLAIM_HOST(card->func);
        sdio_writeb(card->func, 0, REG_SDIO_TX_ALLOC_STATE, &ret);
        SDIO_RELEASE_HOST(card->func);
        cabrio_host_to_card_done(card->priv);
    }
    #endif // ENABLE_TX_DONE_INT

    ret = 0;
out:
    //cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "- %d", received);
    return received;
} // end of - cabrio_rx -


// Write packet to Cabrio
static int cabrio_sdio_write_packet(struct cabrio_sdio_card *card, u8 *packet_data, u32 size)
{
    int               ret  = 0;
    struct sdio_func *func = card->func;
    int               writesize;
    u32               ready_retry_count = 0;
    u32               alloc_retry_count = 0;
    #ifdef RX_IN_WORKER
    u32               received_packet = 0;
    #endif //  RX_IN_WORKER

    writesize = sdio_align_size(func, size);

    // Ask for storage 
    //  - Check HCI ready
    do {
        u8 result;
        SDIO_CLAIM_HOST(func);
        result = sdio_readb(func, REG_SDIO_TX_ALLOC_STATE, &ret);
        SDIO_RELEASE_HOST(func);

        result &= (SDIO_TX_ALLOC_SUCCESS | SDIO_TX_MAILBOX_FULL);
        if (result == 0)
            break;

        ready_retry_count++;
        if (ready_retry_count < 100) {
            #ifdef RX_IN_WORKER
            if (cabrio_rx(card))
                received_packet++;
            #endif // RX_IN_WORKER
            continue;
        }

        // TX allocate keeps high. Previous TX allocation timed out. Send dummy data to clear it.
        if (result == SDIO_TX_ALLOC_SUCCESS) {
            char buf[64];
            memset(buf, 0, sizeof(buf));
            SDIO_CLAIM_HOST(func);
            sdio_memcpy_toio(func, card->data_io_addr, buf, sizeof(buf));
            SDIO_RELEASE_HOST(func);
            pr_err("Clear up previous TX request.\n");
            break;
        }

        pr_err("HCI is busy (%02X).\n", result);
        return (-1);
    } while (1);

    //  - Request packet memory 
    SDIO_CLAIM_HOST(func);
    sdio_writeb(func,
                (writesize + ((1 << SDIO_TX_ALLOC_SIZE_SHIFT) - 1)) >> SDIO_TX_ALLOC_SIZE_SHIFT,
                 REG_SDIO_TX_ALLOC_SIZE, &ret);
    //  - Check request is satisfied.
    do {
        u8 result = sdio_readb(func, REG_SDIO_TX_ALLOC_STATE, &ret);
        if (ret == 0) {
            if ((result & (SDIO_TX_ALLOC_SUCCESS | SDIO_TX_MAILBOX_FULL)) == SDIO_TX_ALLOC_SUCCESS) {
                break;
            } else if (result & SDIO_TX_NO_ALLOC)
                cabrio_dbg_sdio("SDIO_TX_NO_ALLOC\n");
            else if (result & SDIO_TX_DULPICATE_ALLOC)
                cabrio_dbg_sdio("SDIO_TX_DULPICATE_ALLOC\n");
            // else
            {
                #ifdef RX_IN_WORKER
                if (cabrio_rx(card))
                    received_packet++;
                #endif // RX_IN_WORKER

                alloc_retry_count++;
                if (alloc_retry_count < 1000) {
//                    if ((alloc_retry_count & 0x7F) == 0)
//                        msleep(1);
                    continue;
                }
                cabrio_dbg_sdio("POLLING SDIO_TX_ALLOC FAIL (%02x)\n", result);
            }
            ret = -1;
        } else {
            pr_err("Request TX failed. (%d)\n", ret);
        }
        SDIO_RELEASE_HOST(func);
        return ret;
    } while (1);

    ret = sdio_memcpy_toio(func, card->data_io_addr, packet_data, writesize);
    SDIO_RELEASE_HOST(func);

    if (ret /*WARN_ON(ret)*/)
        cabrio_dbg_sdio("Packet write failed (%d)\n", ret);

    cabrio_dbg_sdio("RDY: %d, ALC: %d, sz: %u, blk: %u\n", ready_retry_count, alloc_retry_count,
                    writesize,
                    (writesize + ((1 << SDIO_TX_ALLOC_SIZE_SHIFT) - 1)) >> SDIO_TX_ALLOC_SIZE_SHIFT);

    #ifdef RX_IN_WORKER
    return (received_packet > 0)  ? received_packet : ret;
    #else
    return ret;
    #endif // RX_IN_WORKER
} // end of - cabrio_sdio_write_packet -


// Read packet from Cabrio
int cabrio_sdio_read_packet(struct cabrio_sdio_card *card, u8 *packet_data, u32 *size)
{
    int ret = 0;
    struct sdio_func *func = card->func;
    u32 read_size;

    *size = 0;
#if 0
    _cbrio_sdio_get_irq(card, &status);

    if ( !(status & 0x1 ) ) {
        cabrio_dbg_sdio("No RX data\n");
        return -1;
    }
#endif // 0
    //Read RX packet length
    SDIO_CLAIM_HOST(func);

    read_size = (u32)sdio_readb(func, REG_CARD_PKT_LEN_0, &ret);
    if (ret == 0)
        read_size = read_size | ((u32)sdio_readb(func, REG_CARD_PKT_LEN_1, &ret) << 8);

    SDIO_RELEASE_HOST(func);
    //BUG_ON(*size > 0xff00);
    
    // Read RX data
    if ((read_size > 0) && (ret == 0)) {
        ret = _cabrio_sdio_read_data(card, packet_data, read_size);
        *size = read_size;
        cabrio_dbg_sdio("RX %d\n", read_size);
    } else
        *size = 0;
    return ret;
}


// Write to Cabrio's SRAM 
int cabrio_sdio_write_sram(struct cabrio_sdio_card *card, u32 addr, u8 *data, u32 size)
{
    int     ret = 0;
    struct  sdio_func *func = card->func;
    u32     writesize;

    SDIO_CLAIM_HOST(func);

    do {
        u8   sdio_data[8];
        // Set destination address
        // 0xC0000860 is the DMA target address
        sdio_data[0] = (u8)0x60;
        sdio_data[1] = (u8)0x08;
        sdio_data[2] = (u8)0x00;
        sdio_data[3] = (u8)0xC0;
        // Target address
        sdio_data[4] = (u8)addr;
        sdio_data[5] = (u8)(addr >> 8);
        sdio_data[6] = (u8)(addr >> 16);
        sdio_data[7] = (u8)(addr >> 24);

        ret = sdio_memcpy_toio(func, card->reg_io_addr, sdio_data, 8);
        if (unlikely(ret)) break;

        // Set data path to DMA to SRAM
        sdio_writeb(func, 2, 0x0c, &ret);
        if (unlikely(ret)) break;

        // Write to SRAM
        writesize = sdio_align_size(func, size);
        /*
        cabrio_dbg_sdio("Write %d to SRAM %08X: %08X %08X %08X %08X ...", writesize, addr,
                        data32[0], data32[1], data32[2], data32[3]);
        */
        ret = sdio_memcpy_toio(func, card->data_io_addr, data, writesize);
        if (unlikely(ret)) return ret;

        // Set data path back to packet
        sdio_writeb(func, 0, 0x0c, &ret);
        if (unlikely(ret)) return ret;
    }while (0);

    SDIO_RELEASE_HOST(func);

    return ret;
}

// Read from Cabrio's SRAM
int cabrio_sdio_read_sram(struct cabrio_sdio_card *card, u32 addr, u8 *data, u32 size)
{
    int     ret = 0;
    struct  sdio_func *func = card->func;
    u32     readsize;

    SDIO_CLAIM_HOST(func);

    do {
        // Set destination address
        sdio_writeb(func, (u8)(addr & 0x0FF), 0x60, &ret); 
        if (unlikely(ret)) break;
        sdio_writeb(func, (u8)((addr >> 8) & 0x0FF), 0x61, &ret);
        if (unlikely(ret)) break;
        sdio_writeb(func, (u8)((addr >> 16) & 0x0FF), 0x62, &ret);
        if (unlikely(ret)) break;
        sdio_writeb(func, (u8)((addr >> 24) & 0x0FF), 0x63, &ret);
        if (unlikely(ret)) break;

        // Set data path to SRAM
        sdio_writeb(func, 2, 0x0c, &ret);
        if (unlikely(ret)) break;

        // Write to SRAM
        readsize = sdio_align_size(func, size);

        ret = sdio_memcpy_fromio(func, data, card->data_io_addr, readsize);
        if (unlikely(ret)) return ret;

        // Set data path to packet
        sdio_writeb(func, 0, 0x0c, &ret);
        if (unlikely(ret)) return ret;
    }while (0);

    SDIO_RELEASE_HOST(func);
    return ret;
}


/********************************************************************/
/* I/O                                                              */
/********************************************************************/
#if ORIG
/*
 *  For SD8385/SD8686, this function reads firmware status after
 *  the image is downloaded, or reads RX packet length when
 *  interrupt (with CABRIO_SDIO_H_INT_UPLD bit set) is received.
 *  For SD8688, this function reads firmware status only.
 */
// SSV ??
static u16 cabrio_sdio_read_scratch(struct cabrio_sdio_card *card, int *err)
{
    int ret;
    u16 scratch;

    scratch = sdio_readb(card->func, card->scratch_reg, &ret);
    if (!ret)
        scratch |= sdio_readb(card->func, card->scratch_reg + 1,
                    &ret) << 8;

    if (err)
        *err = ret;

    if (ret)
        return 0xffff;

    return scratch;
}


static u8 cabrio_sdio_read_rx_unit(struct cabrio_sdio_card *card)
{
    int ret;
    u8 rx_unit;

    rx_unit = sdio_readb(card->func, CABRIO_SDIO_RX_UNIT, &ret);

    if (ret)
        rx_unit = 0;

    return rx_unit;
}

static u16 cabrio_sdio_read_rx_len(struct cabrio_sdio_card *card, int *err)
{
    int ret;
    u16 rx_len;

    switch (card->model) {
    case MODEL_6200:
        rx_len = cabrio_sdio_read_scratch(card, &ret);
        break;
    default: /* for newer chipsets */
        break;
    }

    if (err)
        *err = ret;

    return rx_len;
}
#endif // ORIG

static int cabrio_sdio_handle_cmd(struct cabrio_sdio_card *card,
        u8 *buffer, unsigned size)
{
    struct cabrio_private *priv = card->priv;
    int ret;
    unsigned long flags;
    u8 i;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    if (size > CABRIO_CMD_BUFFER_SIZE) {
        cabrio_dbg_sdio("response packet too large (%d bytes)\n",
            (int)size);
        ret = -E2BIG;
        goto out;
    }

    spin_lock_irqsave(&priv->driver_lock, flags);

    i = (priv->resp_idx == 0) ? 1 : 0;
    BUG_ON(priv->resp_len[i]);
    priv->resp_len[i] = size;
    memcpy(priv->resp_buf[i], buffer, size);
    cabrio_notify_command_response(priv, i);

    spin_unlock_irqrestore(&card->priv->driver_lock, flags);

    ret = 0;

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);
    return ret;
}


#if ORIG
static int cabrio_sdio_handle_cmd_orig(struct cabrio_sdio_card *card,
        u8 *buffer, unsigned size)
{
    struct cabrio_private *priv = card->priv;
    int ret;
    unsigned long flags;
    u8 i;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    if (size > CABRIO_CMD_BUFFER_SIZE) {
        cabrio_dbg_sdio("response packet too large (%d bytes)\n",
            (int)size);
        ret = -E2BIG;
        goto out;
    }

    spin_lock_irqsave(&priv->driver_lock, flags);

    i = (priv->resp_idx == 0) ? 1 : 0;
    BUG_ON(priv->resp_len[i]);
    priv->resp_len[i] = size;
    memcpy(priv->resp_buf[i], buffer, size);
    cabrio_notify_command_response(priv, i);

    spin_unlock_irqrestore(&card->priv->driver_lock, flags);

    ret = 0;

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);
    return ret;
}
#endif // ORIG


static int cabrio_sdio_handle_data(struct cabrio_sdio_card *card,
        u8 *buffer, unsigned size)
{
    int ret;
    struct sk_buff *skb;
    char *data;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    if (size > CABRIO_ETH_RX_PACKET_BUFFER_SIZE) {
        cabrio_dbg_sdio("response packet too large (%d bytes)\n",
            (int)size);
        ret = -E2BIG;
        goto out;
    }

    skb = dev_alloc_skb(CABRIO_ETH_RX_PACKET_BUFFER_SIZE + NET_IP_ALIGN);
    if (!skb) {
        ret = -ENOMEM;
        goto out;
    }

    skb_reserve(skb, NET_IP_ALIGN);

    data = skb_put(skb, size);

    memcpy(data, buffer, size);

    cabrio_process_rxed_packet(card->priv, skb);

    ret = 0;

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);

    return ret;
}


static int cabrio_sdio_handle_event(struct cabrio_sdio_card *card,
        u8 *buffer, unsigned size)
{
    int ret = 0;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    cabrio_queue_event(card->priv, buffer, size);
    //cabrio_dbg_sdio("Event received.\n");

//out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);

    return ret;
}


#if ORIG
static int cabrio_sdio_handle_event_orig(struct cabrio_sdio_card *card,
        u8 *buffer, unsigned size)
{
    int ret;
    u32 event;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    if (card->model == MODEL_6200) {
        event = sdio_readb(card->func, CABRIO_SDIO_EVENT, &ret);
        if (ret)
            goto out;

        /* right shift 3 bits to get the event id */
        event >>= 3;
    } else {
        if (size < 4) {
            cabrio_dbg_sdio("event packet too small (%d bytes)\n",
                (int)size);
            ret = -EINVAL;
            goto out;
        }
        event = buffer[3] << 24;
        event |= buffer[2] << 16;
        event |= buffer[1] << 8;
        event |= buffer[0] << 0;
    }

    cabrio_queue_event(card->priv, event & 0xFF);
    ret = 0;

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);

    return ret;
}


static int cabrio_sdio_wait_status(struct cabrio_sdio_card *card, const u8 condition)
{
    u8 status;
    unsigned long timeout;
    int ret = 0;

    timeout = jiffies + HZ;
    while (1) {
        #if 1
        CABRIO_TODO(__func__);
        ret = -1;
        #else
        status = sdio_readb(card->func, CABRIO_SDIO_STATUS, &ret);
        #endif
        if (ret)
            return ret;
        if ((status & condition) == condition)
            break;
        if (time_after(jiffies, timeout))
            return -ETIMEDOUT;
        mdelay(1);
    }
    return ret;
}
#endif // ORIG


static int cabrio_sdio_card_to_host(struct cabrio_sdio_card *card)
{
    int ret;
    u32 size;
    struct cfg_host_rxpkt *rxpkt = (struct cfg_host_rxpkt *)card->buffer;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    ret = cabrio_sdio_read_packet(card, card->buffer, &size);

    if (ret)
        goto out;

    BUG_ON(rxpkt->len < size);
    #if 1
    // Parse RX frame and execute corresponding functions.
    switch (rxpkt->c_type)
    {
        //-------------
        //RX
    case M0_RXEVENT:
        cabrio_sdio_handle_data(card, card->buffer, rxpkt->len);
        break;
    case HOST_EVENT:
        {
        HDR_HostEvent *p_host_evt = (HDR_HostEvent *)card->buffer;

        if (p_host_evt->h_event == SOC_EVT_CMD_RESP)
            cabrio_sdio_handle_cmd(card, card->buffer, size);
        else
            cabrio_sdio_handle_event(card, card->buffer, size);
        }
        break;
    default:
        cabrio_dbg_sdio("Unexpect c_type %d appeared", rxpkt->c_type);
        ret = -1;
    }
    #else
    u16 type, chunk;
    
    chunk = card->buffer[0] | (card->buffer[1] << 8);
    type = card->buffer[2] | (card->buffer[3] << 8);

    cabrio_dbg_sdio("packet of type %d and size %d bytes\n",
        (int)type, (int)chunk);

    if (chunk > size) {
        cabrio_dbg_sdio("packet fragment (%d > %d)\n",
            (int)chunk, (int)size);
        ret = -EINVAL;
        goto out;
    }

    if (chunk < size) {
        cabrio_dbg_sdio("packet fragment (%d < %d)\n",
            (int)chunk, (int)size);
    }

    switch (type) {
    case CABRIO_MS_CMD:
        ret = cabrio_sdio_handle_cmd(card, card->buffer + 4, chunk - 4);
        if (ret)
            goto out;
        break;
    case CABRIO_MS_DAT:
        ret = cabrio_sdio_handle_data(card, card->buffer + 4, chunk - 4);
        if (ret)
            goto out;
        break;
    case CABRIO_MS_EVENT:
        ret = cabrio_sdio_handle_event(card, card->buffer + 4, chunk - 4);
        if (ret)
            goto out;
        break;
    default:
        cabrio_dbg_sdio("invalid type (%d) from firmware\n",
                (int)type);
        ret = -EINVAL;
        goto out;
    }
    #endif
out:
    if (ret)
        pr_err("problem fetching packet from firmware\n");

    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);

    return ret;
}


static void cabrio_sdio_host_to_card_worker (struct work_struct *work)
{
    struct cabrio_sdio_card *card = container_of(work, struct cabrio_sdio_card, packet_worker);
    #if 0
    cabrio_dbg_net("cabrio_sdio_host_to_card_worker\n");
    cabrio_host_to_card_done(card->priv);
    #else
    struct cabrio_sdio_packet *packet;
    struct cabrio_sdio_packet *next_packet = NULL;
    int ret;
    unsigned long flags;
    u32 remain_packet_num = (-1);
    u32 sent_packet = 0;
    #ifdef RX_IN_WORKER
    s32 received;
    u32 received_packet = 0;
    #endif // RX_IN_WORKER
    u32 loop_count = 0;
    #ifdef CHECK_HCI_INQ_FULL
    u32 hci_iq_full = 0;
    #endif // CHECK_HCI_INQ_FULL

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    // card = container_of(work, struct cabrio_sdio_card, packet_worker);
    spin_lock_irqsave(&card->lock, flags);
    if (card->in_worker) {
        //pr_err("#1\n");
        spin_unlock_irqrestore(&card->lock, flags);
        cabrio_dbg_leave(CABRIO_DBG_SDIO);
        card->en_worker = false;
        return;
    }
    //pr_err("#2\n");
    card->in_worker = true;
    spin_unlock_irqrestore(&card->lock, flags);

    while (loop_count < MAX_TX_PACKET_COUNT) {
        #ifdef RX_IN_WORKER
        if ((received = cabrio_rx(card)))
            received_packet++;
        #endif // RX_IN_WORKER

        //spin_lock_irqsave(&card->lock, flags);
        //pr_err("#3\n");
        #ifdef CHECK_HCI_INQ_FULL
        if ((hci_iq_full = is_hci_iq_full(card)))
            break;
        #endif //  CHECK_HCI_INQ_FULL

        spin_lock_irqsave(&card->lock, flags);
        //pr_err("#4\n");
        packet = card->packets;
        if (packet) {
            card->packets = packet->next;
            card->packet_num--;
            remain_packet_num = card->packet_num;
        }
        spin_unlock_irqrestore(&card->lock, flags);
        //pr_err("#5\n");

        if (!packet) {
            //spin_unlock_irqrestore(&card->lock, flags);
            next_packet = NULL;
        } else {
            next_packet = packet->next;

            sent_packet++;

            // Write packet to card
            //pr_err("#6\n");
            ret = cabrio_sdio_write_packet(card, packet->buffer, packet->nb);

            if (ret < 0)
                pr_err("Failed %d sending packet to " DRV_NAME ".\n", ret);
            #ifdef RX_IN_WORKER
            else if (ret > 0) {
                received_packet += ret;
                received = 1;
            }
            #endif // RX_IN_WORKER
            //pr_err("#7\n");
            kfree(packet);
            //spin_unlock_irqrestore(&card->lock, flags);
        }

        if (   (next_packet == NULL)
            #ifdef RX_IN_WORKER
            && (received == 0)
            #endif // RX_IN_WORKER
           )
            break;
        loop_count++;
    }

    #ifndef ENABLE_TX_DONE_INT
    if (sent_packet > 0)
        cabrio_host_to_card_done(card->priv);
    #endif //ENABLE_TX_DONE_INT

    if (   (next_packet == NULL)
        #ifdef RX_IN_WORKER
        && (received == 0)
        #endif // RX_IN_WORKER
        #ifdef CHECK_HCI_INQ_FULL
        && (!hci_iq_full)
        #endif // CHECK_HCI_INQ_FULL
        ) {
        // #ifdef RX_IN_WORKER
        #if 0
        int               err_ret = 0;
        // Enable RX ready interrupt
        SDIO_CLAIM_HOST(card->func);
        #ifdef ENABLE_TX_DONE_INT
        sdio_writeb(card->func, 0x08, REG_INT_MASK, &err_ret);
        #else
        sdio_writeb(card->func, 0x08|0x02, REG_INT_MASK, &err_ret);
        #endif // ENABLE_TX_DONE_INT
        SDIO_RELEASE_HOST(card->func);
        #endif // RX_IN_WORKER
        card->en_worker = false;
    } else
        queue_work(card->workqueue, &card->packet_worker);

    //pr_err("#8\n");
    spin_lock_irqsave(&card->lock, flags);
    //pr_err("#9\n");
    card->in_worker = false;
    spin_unlock_irqrestore(&card->lock, flags);
    //pr_err("#10\n");

    #ifdef RX_IN_WORKER
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "     %u - %u - %u\n", remain_packet_num, sent_packet, received_packet);
    #else
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "     %u - %u \n", remain_packet_num, sent_packet);
    #endif // RX_IN_WORKER
    #endif // 0
} // end of - cabrio_sdio_host_to_card_worker -


/********************************************************************/
/* Firmware                                                         */
/********************************************************************/
#if ORIG
#define FW_DL_READY_STATUS (CABRIO_SDIO_IO_RDY | CABRIO_SDIO_DL_RDY)

static int cabrio_sdio_prog_real(struct cabrio_sdio_card *card,
                                 const struct firmware *fw)
{
    int ret;
    unsigned long timeout;
    u8 *chunk_buffer;
    u32 chunk_size;
    const u8 *firmware;
    size_t size, req_size;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    chunk_buffer = kzalloc(512, GFP_KERNEL);
    if (!chunk_buffer) {
        ret = -ENOMEM;
        goto out;
    }

    SDIO_CLAIM_HOST(card->func);

    ret = sdio_set_block_size(card->func, 32);
    if (ret)
        goto release;

    firmware = fw->data;
    size = fw->size;

    while (size) {
        #if 1
        CABRIO_TODO(__func__);
        size = 0;
        #else
        ret = cabrio_sdio_wait_status(card, FW_DL_READY_STATUS);
        if (ret)
            goto release;

        req_size = sdio_readb(card->func, CABRIO_SDIO_RD_BASE, &ret);
        if (ret)
            goto release;

        req_size |= sdio_readb(card->func, CABRIO_SDIO_RD_BASE + 1, &ret) << 8;
        if (ret)
            goto release;
/*
        cabrio_dbg_sdio("firmware wants %d bytes\n", (int)req_size);
*/
        if (req_size == 0) {
            cabrio_dbg_sdio("firmware helper gave up early\n");
            ret = -EIO;
            goto release;
        }

        if (req_size & 0x01) {
            cabrio_dbg_sdio("firmware helper signalled error\n");
            ret = -EIO;
            goto release;
        }

        if (req_size > size)
            req_size = size;

        while (req_size) {
            chunk_size = min(req_size, (size_t)512);

            memcpy(chunk_buffer, firmware, chunk_size);
/*
            cabrio_dbg_sdio("sending %d bytes (%d bytes) chunk\n",
                chunk_size, (chunk_size + 31) / 32 * 32);
*/
            ret = sdio_writesb(card->func, card->ioport,
                chunk_buffer, roundup(chunk_size, 32));
            if (ret)
                goto release;

            firmware += chunk_size;
            size -= chunk_size;
            req_size -= chunk_size;
        }
        #endif
    }

    ret = 0;

    cabrio_dbg_sdio("waiting for firmware to boot...\n");

    /* wait for the firmware to boot */
    timeout = jiffies + HZ;
    while (1) {
        u16 scratch;

        scratch = cabrio_sdio_read_scratch(card, &ret);
        if (ret)
            goto release;

        if (scratch == CABRIO_SDIO_FIRMWARE_OK)
            break;

        if (time_after(jiffies, timeout)) {
            ret = -ETIMEDOUT;
            goto release;
        }

        msleep(10);
    }

    ret = 0;

release:
    SDIO_RELEASE_HOST(card->func);
    kfree(chunk_buffer);

out:
    if (ret)
        pr_err("failed to load firmware\n");

    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);
    return ret;
}
#endif // ORIG

#define BUF_SIZE   (FW_BLOCK_SIZE)
static int cabrio_sdio_prog_firmware(struct cabrio_sdio_card *card)
{
    int ret = 0;
    const struct firmware *cabrio_fw = NULL;
    u32   sram_addr = 0x00000000;
    #ifdef ENABLE_FW_SELF_CHECK
    u32   reg_value;
    u32   checksum = FW_CHECKSUM_INIT;
    u32   fw_checksum;
    u32   block_count = 0;
    u32   block_idx = 0;
    u32   res_size;
    u8   *final_buffer = NULL;
    u8   *fw_data;
    u32   retry_count = 3;
    u32  *fw_data32;
    u32   total_block_count;
    #else
    u32   retry_count = 1;
    #endif

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    // Load firmware
    ret = cabrio_get_firmware(&card->func->dev, &cabrio_fw_name,
                                  card->model, &fw_table[0], &cabrio_fw);
    if (ret) {
        pr_err("failed to find firmware (%d)\n", ret);
        goto out;
    }

    #ifdef ENABLE_FW_SELF_CHECK
    block_count = cabrio_fw->size / FW_BLOCK_SIZE;
    total_block_count = block_count;
    res_size = cabrio_fw->size % FW_BLOCK_SIZE;
    // Accumulate checksum for complete blocks
    {
    int        word_count = (int)(block_count * FW_BLOCK_SIZE / sizeof(u32));
    int     i;
    fw_data32 = (u32 *)cabrio_fw->data;
    for (i = 0; i < word_count; i++) {
        checksum += fw_data32[i];
    }
    }
    // For incomplete block, fill with 0xA5 to FW_BLOCK_SIZE.
    if (res_size > 0) {
        int        word_count = (int)(FW_BLOCK_SIZE / sizeof(u32));
        int     i;
        final_buffer = kzalloc(FW_BLOCK_SIZE, GFP_KERNEL);
        if (final_buffer == NULL) {
            pr_err("failed to find firmware (%d)\n", ret);
            goto out;
        }
        memcpy(final_buffer, &cabrio_fw->data[block_count * FW_BLOCK_SIZE], res_size);
        memset(&final_buffer[res_size], 0xA5, FW_BLOCK_SIZE - res_size);
        // Accumulate checksum for the incomplete block
        fw_data32 = (u32 *)final_buffer;
        for (i = 0; i < word_count; i++) {
            checksum += fw_data32[i];
        }
        // Total block number for download.
        ++total_block_count;
    }
    // Calculate the final checksum.
    checksum = ((checksum >> 24) + (checksum >> 16) + (checksum >> 8) + checksum) & 0x0FF;
    checksum <<= 16;
    #endif // ENABLE_FW_SELF_CHECK

    do {
        // Reset CPU
        cabrio_sdio_write_reg(card, ADR_BRG_SW_RST, 0x0);
        cabrio_sdio_write_reg(card, ADR_BOOT, 0x01);

        // Write firmware to SRAM address 0
        #ifdef ENABLE_FW_SELF_CHECK
        cabrio_dbg_sdio("Writing %d blocks to Cabrio...", block_count);
        for (block_idx = 0, fw_data = (u8 *)cabrio_fw->data, sram_addr = 0;
             block_idx < block_count;
             block_idx++, fw_data += FW_BLOCK_SIZE, sram_addr += FW_BLOCK_SIZE) {
            ret = cabrio_sdio_write_sram(card, sram_addr, fw_data, FW_BLOCK_SIZE);
            if (ret)
                break;
        }

        if ((ret == 0) && (res_size > 0)) {
            cabrio_dbg_sdio("Writing final blocks to Cabrio...");
            ret = cabrio_sdio_write_sram(card, sram_addr,
                                         final_buffer, FW_BLOCK_SIZE);
        }
        #else // ENABLE_FW_SELF_CHECK
        ret = cabrio_sdio_write_sram(card, 0, (u8 *)cabrio_fw->data, cabrio_fw->size);
        #endif // ENABLE_FW_SELF_CHECK
    
        if (ret == 0) {
            #ifdef ENABLE_FW_SELF_CHECK
            // Inform FW that how many blocks is downloaded such that FW can calculate the checksum.
            cabrio_sdio_read_reg(card, SD_REG_BASE + 0x10, &reg_value);
            reg_value &= 0xFF00FFFF;
            cabrio_sdio_write_reg(card, SD_REG_BASE + 0x10, reg_value | (total_block_count << 16));
            #endif // ENABLE_FW_SELF_CHECK

            // Release reset to let CPU run.
            cabrio_sdio_write_reg(card, ADR_BRG_SW_RST, 0x1);

            cabrio_dbg_sdio("Firmware \"%s\" loaded\n", cabrio_fw_name);
            #ifdef ENABLE_FW_SELF_CHECK
            // Wait FW to calculate checksum.
            msleep(50);
            // Check checksum result and set to complement value if checksum is OK.
            cabrio_sdio_read_reg(card, SD_REG_BASE + 0x10, &fw_checksum);
            fw_checksum = fw_checksum & 0x00FF0000;
            if (fw_checksum == checksum) {
                cabrio_sdio_write_reg(card, SD_REG_BASE + 0x10, reg_value | (~checksum & 0x00FF0000));
                ret = 0;
                cabrio_dbg_sdio("Firmware check OK.\n");
                break;
            } else {
                cabrio_dbg_sdio("FW checksum error: %04x != %04x\n", fw_checksum, checksum);
                ret = -1;
            }
            #endif
        } else {
            cabrio_dbg_sdio("Firmware \"%s\" download failed.\n", cabrio_fw_name);
            ret = -1;
        }
    } while (--retry_count);

    if (ret)
        goto out;
// success:
    SDIO_CLAIM_HOST(card->func);
    sdio_set_block_size(card->func, CABRIO_SDIO_BLOCK_SIZE);
    SDIO_RELEASE_HOST(card->func);
    ret = 0;

    card->priv->fw_ready = 1;

out:
    if (cabrio_fw)
        release_firmware(cabrio_fw);
    #ifdef ENABLE_FW_SELF_CHECK
    if (final_buffer != NULL)
        kfree(final_buffer);
    #endif
    // Sleep to let Cabrio get ready.
    msleep(50);
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);
    return ret;
}

/********************************************************************/
/* Power management                                                 */
/********************************************************************/
#if 0
static int cabrio_power_on_init (struct cabrio_sdio_card *card)
{
    int ret = -1;
    do {
        // Initialize RF
        if (cabrio_sdio_write_reg(card, 0xcb110000, 0x5F00EFAE)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110004, 0x00001FC0)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110008, 0x1C96CA3A)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11000c, 0x15155A74)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110010, 0x01011A88)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110014, 0x3CBF703C)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110018, 0x00057579)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11001c, 0x000103A7)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110020, 0x000103A6)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110024, 0x00012001)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110028, 0x00036000)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11002c, 0x00000CA8)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110030, 0x002A0224)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110034, 0x00001E55)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110038, 0x00007C7C)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11003c, 0x55666666)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110040, 0x005508F8)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110044, 0x07C08BFF)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110048, 0xF1111A27)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11004c, 0x2773F53C)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110050, 0x00000A7C)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110054, 0x00087FF8)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110058, 0x00103014)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11005c, 0x0000848A)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110060, 0x00406030)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110064, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110068, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11006c, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110070, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110074, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110078, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11007c, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110080, 0x00820820)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110084, 0x00004080)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110088, 0x00003EAA)) break;
        if (cabrio_sdio_write_reg(card, 0xcb11008c, 0x5E00FFEB)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110090, 0xAAAAAAAA)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110094, 0x0000243F)) break;
        if (cabrio_sdio_write_reg(card, 0xcb110098, 0x00018B10)) break;
        if (cabrio_sdio_write_reg(card, 0xcb120080, 0x00000000)) break;
        if (cabrio_sdio_write_reg(card, 0xcb120084, 0x00000000)) break;
        if (cabrio_sdio_write_reg(card, 0xcb120088, 0x00000000)) break;
        if (cabrio_sdio_write_reg(card, 0xcb120090, 0x00000813)) break;
        if (cabrio_sdio_write_reg(card, 0xcb120094, 0x00000000)) break;
        if (cabrio_sdio_write_reg(card, 0xcb1203f8, 0xFF000000)) break;
        // Switch clock to PLL output of RF
        if (cabrio_sdio_write_reg(card, 0xc0000018, 0x00000003)) break;
        ret = 0;
    } while (0);
    return ret;
} // end of - cabrio_power_on_init -
#endif // 0

static void _send_Cabrio_command (struct cabrio_sdio_card *card, u32 cmd_id, const u8 *cmd_data, u32 cmd_length)
{
    u32              cmd_size = cmd_length + HOST_CMD_HDR_LEN;
    u32              event_size = sizeof(HDR_HostEvent) + sizeof(struct resp_evt_result);
    u8              *buf = (u8 *)kzalloc(cmd_size + event_size, GFP_KERNEL);
    HDR_HostCmd     *cmd = (HDR_HostCmd *)buf;
    HDR_HostEvent   *event = (HDR_HostEvent *)(buf + cmd_size);

    if (cmd == NULL)
        return;

    cmd->h_cmd = cmd_id;
    cmd->len = cmd_size;
    cmd->c_type = HOST_CMD;
    memcpy(cmd->dat8, cmd_data, cmd_length);
    #if 0
    if (cmd_length > 7)
    cabrio_dbg_sdio("CMD: -- %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    ((u8 *)cmd)[cmd_size - 8], ((u8 *)cmd)[cmd_size - 7],
                    ((u8 *)cmd)[cmd_size - 6], ((u8 *)cmd)[cmd_size - 5],
                    ((u8 *)cmd)[cmd_size - 4], ((u8 *)cmd)[cmd_size - 3],
                    ((u8 *)cmd)[cmd_size - 2], ((u8 *)cmd)[cmd_size - 1]);
    #endif // 0
    event->len = event_size;

    cabrio_cmd_with_response(card->priv, cmd_id, cmd, event);
    //cabrio_cmd_async(card->priv, cmd_id, cmd, cmd_size);
    kfree(cmd);
}


static int _init_Cabrio (struct cabrio_sdio_card *card)
{
    int ret = 0; //(-1);
    
    #define _cabrio_write_reg(addr, value) \
        if ((ret = cabrio_sdio_write_reg(card, addr, value))) break;
        
    #define _cabrio_load_fw(firmware_name) \
        if ((ret = cabrio_sdio_prog_firmware(card))) break;

    #define _command_Cabrio(cmd_id, cmd_data, cmd_length) \
        _send_Cabrio_command(card, cmd_id, cmd_data, cmd_length)

#if 0
    #define _command_Cabrio(cmd_id, cmd_data, cmd_length) \
        do { \
            u32              cmd_size = cmd_length + HOST_CMD_HDR_LEN; \
            HDR_HostCmd     *cmd = (HDR_HostCmd *)kzalloc(cmd_size, GFP_KERNEL); \
            \
            cmd->h_cmd = cmd_id; \
            cmd->len = cmd_size; \
            cmd->c_type = HOST_CMD; \
            memcpy(cmd->dat8, cmd_data, cmd_length); \
            cabrio_cmd_async(card->priv, cmd_id, cmd, cmd_size); \
            kfree(cmd); \
        } while (0)
#endif
    do {
        #include "cabrio_init.h"
    } while (0);

    return ret;
} // end of - _init_Cabrio -


static int cabrio_sdio_power_on(struct cabrio_sdio_card *card)
{
    struct sdio_func *func = card->func;
    int ret;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    SDIO_CLAIM_HOST(func);
    // IO Enable
    ret = sdio_enable_func(func);
    if (ret)
        goto release;

    ret = sdio_claim_irq(func, cabrio_sdio_interrupt);
    if (ret)
        goto disable;
#if 0
    ret = mmc_io_rw_direct(func->card, 0, 0, SDIO_CCCR_IENx, 0, &reg);
    if (ret)
        goto disable;

    reg |= 1 << func->num;

    reg |= 1; /* Master interrupt enable */

    ret = mmc_io_rw_direct(func->card, 1, 0, SDIO_CCCR_IENx, reg, NULL);
    if (ret)
        goto disable;

    reg = 1 << func->num;
    ret = mmc_io_rw_direct(func->card, 1, 0, SDIO_CCCR_IOEx, reg, NULL);
    if (ret)
        goto disable;
#endif
    /*
     * Enable interrupts now that everything is set up
     */
    // bit0: RX, bit1: TX done, bit2: OOB enable
    #ifdef ENABLE_TX_DONE_INT
    sdio_writeb(func, 0x08, REG_INT_MASK, &ret);
    #else
    sdio_writeb(func, 0x08 | 0x02, REG_INT_MASK, &ret);
    #endif // ENABLE_TX_DONE_INT

    if (ret)
        goto release_irq;

    SDIO_RELEASE_HOST(func);

    ret = _init_Cabrio(card);

    cabrio_dbg_sdio("Cabrio powered up.\n");

    goto out;
release_irq:
    sdio_release_irq(func);
disable:
    sdio_disable_func(func);
release:
    SDIO_RELEASE_HOST(func);
out:
    cabrio_dbg_leave(CABRIO_DBG_SDIO);
    return ret;
}


static int cabrio_sdio_power_off(struct cabrio_sdio_card *card)
{
    struct sdio_func *func = card->func;
    struct cabrio_private *priv = card->priv;

    priv->fw_ready = 0;

    sdio_claim_host(func);
    sdio_release_irq(func);
    sdio_disable_func(func);
    sdio_release_host(func);
    return 0;
}


/*******************************************************************/
/* Cabrio callbacks                                                */
/*******************************************************************/

static int cabrio_sdio_host_to_card(struct cabrio_private *priv,
        u8 type, u8 *buf, u16 nb)
{
    int ret;
    struct cabrio_sdio_card *card;
    struct cabrio_sdio_packet *packet, *cur;
    u16 size;
    unsigned long flags;
    u32 cur_packet_num;

    cabrio_dbg_enter_args(CABRIO_DBG_SDIO, "type %d, bytes %d", type, nb);

    card = priv->card;

    if (nb > (65536 - sizeof(struct cabrio_sdio_packet) - 4)) {
        ret = -EINVAL;
        goto out;
    }

    /*
     * The transfer must be in one transaction or the firmware
     * goes suicidal. There's no way to guarantee that for all
     * controllers, but we can at least try.
     */
    size = sdio_align_size(card->func, nb/* CABRIO --  + 4 */);

    packet = kzalloc(sizeof(struct cabrio_sdio_packet) + size,
            GFP_ATOMIC);
    if (!packet) {
        ret = -ENOMEM;
        goto out;
    }

    packet->next = NULL;
    packet->nb = size;

    /*
     * SDIO specific header.
     */
    /*CABRIO -- 
    packet->buffer[0] = (nb + 4) & 0xff;
    packet->buffer[1] = ((nb + 4) >> 8) & 0xff;
    packet->buffer[2] = type;
    packet->buffer[3] = 0;

    memcpy(packet->buffer + 4, buf, nb);
    */
    memcpy(packet->buffer, buf, nb);
    spin_lock_irqsave(&card->lock, flags);
    cur_packet_num = card->packet_num;

    if (!card->packets)
        card->packets = packet;
    else {
        cur = card->packets;
        while (cur->next)
            cur = cur->next;
        cur->next = packet;
    }
    ++card->packet_num;

    switch (type) {
    case CABRIO_MS_CMD:
        priv->dnld_sent = DNLD_CMD_SENT;
        break;
    case CABRIO_MS_DAT:
        priv->dnld_sent = DNLD_DATA_SENT;
        break;
    default:
        cabrio_dbg_sdio("unknown packet type %d\n", (int)type);
    }

    spin_unlock_irqrestore(&card->lock, flags);

    // Only queue work when there is no packets in queue.
#if 0
    if (cur_packet_num == 0) {
#else
    if (card->en_worker == false) {
        card->en_worker = true;
#endif
        queue_work(card->workqueue, &card->packet_worker);
    }

    ret = (card->packet_num >= MAX_QUEUED_PACKET) ? MAX_QUEUED_PACKET : 0;

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d - %u", ret, card->packet_num);

    return ret;
}

static int cabrio_sdio_enter_deep_sleep(struct cabrio_private *priv)
{
    int ret = -1;
    struct cmd_header cmd;

    memset(&cmd, 0, sizeof(cmd));

    cabrio_dbg_sdio("send DEEP_SLEEP command\n");
    // SSV TODO
    #if 0
    ret = __cabrio_cmd(priv, CMD_802_11_DEEP_SLEEP, &cmd, sizeof(cmd),
            cabrio_cmd_copyback, (unsigned long) &cmd);
    #else
    CABRIO_TODO(__func__);
    #endif
    if (ret)
        netdev_err(priv->dev, "DEEP_SLEEP cmd failed\n");

    mdelay(200);
    return ret;
}

static int cabrio_sdio_exit_deep_sleep(struct cabrio_private *priv)
{
    struct cabrio_sdio_card *card = priv->card;
    int ret = -1;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);
    SDIO_CLAIM_HOST(card->func);
    #if 1
    CABRIO_TODO(__func__);
    ret = -1;
    #else
    sdio_writeb(card->func, HOST_POWER_UP, CONFIGURATION_REG, &ret);
    if (ret)
        netdev_err(priv->dev, "sdio_writeb failed!\n");
    #endif
    SDIO_RELEASE_HOST(card->func);
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);
    return ret;
}

static int cabrio_sdio_reset_deep_sleep_wakeup(struct cabrio_private *priv)
{
    struct cabrio_sdio_card *card = priv->card;
    int ret = -1;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);
    SDIO_CLAIM_HOST(card->func);
    // SSV TODO
    #if 0
    sdio_writeb(card->func, 0, CONFIGURATION_REG, &ret);
    if (ret)
        netdev_err(priv->dev, "sdio_writeb failed!\n");
    #else
    CABRIO_TODO(__func__);
    #endif 

    SDIO_RELEASE_HOST(card->func);
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);
    return ret;

}

static struct mmc_host *reset_host;

static void cabrio_sdio_reset_card_worker(struct work_struct *work)
{
    /*
     * The actual reset operation must be run outside of cabrio_thread. This
     * is because mmc_remove_host() will cause the device to be instantly
     * destroyed, and the cabrio driver then needs to end cabrio_thread,
     * leading to a deadlock.
     *
     * We run it in a workqueue totally independent from the cabrio_sdio_card
     * instance for that reason.
     */

    pr_info("Resetting card...");
    #if 0
    mmc_remove_host(reset_host);
    mmc_add_host(reset_host);
    #endif 
}
static DECLARE_WORK(card_reset_work, cabrio_sdio_reset_card_worker);

static void cabrio_sdio_reset_card(struct cabrio_private *priv)
{
    struct cabrio_sdio_card *card = priv->card;

    if (work_pending(&card_reset_work))
        return;

    reset_host = card->func->card->host;
    schedule_work(&card_reset_work);
}

static int cabrio_sdio_power_save(struct cabrio_private *priv)
{
    struct cabrio_sdio_card *card = priv->card;
    int ret;

    flush_workqueue(card->workqueue);

    ret = cabrio_sdio_power_off(card);

    /* Let runtime PM know the card is powered off */
    pm_runtime_put_sync(&card->func->dev);

    return ret;
}

static int cabrio_sdio_power_restore(struct cabrio_private *priv)
{
    int ret = 0;
    struct cabrio_sdio_card *card = priv->card;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    /* Make sure the card will not be powered off by runtime PM */
    pm_runtime_get_sync(&card->func->dev);

    ret = cabrio_sdio_power_on(card);

    cabrio_dbg_leave(CABRIO_DBG_SDIO);

    return ret;
}


/*******************************************************************/
/* SDIO callbacks                                                  */
/*******************************************************************/
#define MAX_CONT_READ_COUNT        (32)
static void cabrio_sdio_interrupt(struct sdio_func *func)
{
    struct cabrio_sdio_card *card = sdio_get_drvdata(func);
#ifndef RX_IN_WORKER
    int ret;
    u8 cause;
    int read_count = MAX_CONT_READ_COUNT; // Contineously read pending RX packets.

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    while (read_count) {
        SDIO_CLAIM_HOST(card->func);
        cause = sdio_readb(card->func, REG_INT_STATUS, &ret);
        sdio_writeb(card->func, ~cause, REG_INT_STATUS, &ret);
        SDIO_RELEASE_HOST(card->func);
        #ifdef ENABLE_TX_DONE_INT
        cause &= (SDIO_RX_READY_INT | SDIO_TX_DONE_INT);
        #else
        cause &= (SDIO_RX_READY_INT);
        #endif
        if (ret || (cause  == 0))
            goto out;

        cabrio_dbg_sdio("interrupt: 0x%X\n", (unsigned)cause);
#if 0
        SDIO_CLAIM_HOST(card->func);
        sdio_writeb(card->func, ~cause, REG_INT_STATUS, &ret);
        SDIO_RELEASE_HOST(card->func);
        if (ret)
            goto out;
#endif
        /*
         * Ignore the define name, this really means the card has
         * successfully received the command.
         */
        card->priv->is_activity_detected = 1;

        if (cause & SDIO_RX_READY_INT) {
            ret = cabrio_sdio_card_to_host(card);
            --read_count;
            if (ret)
                goto out;
        }
        #ifndef ENABLE_TX_DONE_INT
        if (cause & SDIO_TX_DONE_INT) {
            // Clear TX done interrupt.
            SDIO_CLAIM_HOST(card->func);
            sdio_writeb(func, 0, REG_SDIO_TX_ALLOC_STATE, &ret);
            SDIO_RELEASE_HOST(card->func);
            cabrio_host_to_card_done(card->priv);
        }
        #endif // ENABLE_TX_DONE_INT
    }

    ret = 0;

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d - %u", ret, MAX_CONT_READ_COUNT - read_count);
#else // RX_IN_WORKER
    //int err_ret;
    cabrio_dbg_enter(CABRIO_DBG_SDIO);
    // Disable RX interrupt.
    #if 0
    SDIO_CLAIM_HOST(func);
    sdio_writeb(func, 0x08|SDIO_RX_READY_INT|SDIO_TX_DONE_INT, REG_INT_MASK, &err_ret);
    SDIO_RELEASE_HOST(func);
    #endif
    // Schedule worker to handle RX
    queue_work(card->workqueue, &card->packet_worker);
    cabrio_dbg_leave(CABRIO_DBG_SDIO);
#endif // RX_IN_WORKER
}


extern int ssv_devicetype;

static int cabrio_sdio_probe(struct sdio_func *func,
        const struct sdio_device_id *id)
{
    struct cabrio_sdio_card *card;
    struct cabrio_private *priv;
    int ret, i;
    unsigned int model = MODEL_6200;
    struct cabrio_sdio_packet *packet;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    if (ssv_devicetype != 2) {
        pr_err("Not using full MAC driver.\n");
        return -ENODEV;
    }

    if (strcmp(func->card->info[1], "Cabrio")) {
        pr_err("\"%s\" is not SSV6200 device.\n", func->card->info[1]);
        return -ENODEV;
    }

    card = kzalloc(sizeof(struct cabrio_sdio_card), GFP_KERNEL);
    if (!card)
        return -ENOMEM;

    card->func = func;
    card->model = model;

    switch (card->model) {
    case MODEL_6200:
        card->block_size = CABRIO_SDIO_BLOCK_SIZE;
        cabrio_sdio_init(card);
        break;
    default: /* for newer chipsets */
        card->block_size = CABRIO_SDIO_BLOCK_SIZE;
        cabrio_sdio_init(card);
        break;
    }

    spin_lock_init(&card->lock);
    //card->workqueue = create_workqueue("cabrio_fmac_sdio");
    card->workqueue = alloc_workqueue("cabrio_fmac_sdio", WQ_NON_REENTRANT | WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 1);
    INIT_WORK(&card->packet_worker, cabrio_sdio_host_to_card_worker);
    card->in_worker = false;
    card->en_worker = false;

    /* Check if we support this card */
    for (i = 0; i < ARRAY_SIZE(fw_table); i++) {
        if (card->model == fw_table[i].model)
            break;
    }
    if (i == ARRAY_SIZE(fw_table)) {
        pr_err("unknown card model 0x%x\n", card->model);
        ret = -ENODEV;
        goto free;
    }

    sdio_set_drvdata(func, card);

    cabrio_dbg_sdio("class = 0x%X, vendor = 0x%X, "
            "device = 0x%X, model = 0x%X, ioport = 0x%X\n",
            func->class, func->vendor, func->device,
            model, (unsigned)card->ioport);

    priv = cabrio_add_card(card, &func->dev);
    if (!priv) {
        ret = -ENOMEM;
        goto free;
    }

    card->priv = priv;

    priv->card = card;
    priv->hw_host_to_card = cabrio_sdio_host_to_card;
    priv->enter_deep_sleep = cabrio_sdio_enter_deep_sleep;
    priv->exit_deep_sleep = cabrio_sdio_exit_deep_sleep;
    priv->reset_deep_sleep_wakeup = cabrio_sdio_reset_deep_sleep_wakeup;
    priv->reset_card = cabrio_sdio_reset_card;
    priv->power_save = cabrio_sdio_power_save;
    priv->power_restore = cabrio_sdio_power_restore;

    ret = cabrio_sdio_power_on(card);
    if (ret)
        goto err_activate_card;

    ret = cabrio_start_card(priv);
    cabrio_sdio_power_off(card);
    if (ret)
        goto err_activate_card;

    /* Tell PM core that we don't need the card to be powered now */
    pm_runtime_put_noidle(&func->dev);

out:
    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);

    return ret;

err_activate_card:
    flush_workqueue(card->workqueue);
    cabrio_remove_card(priv);
free:
    destroy_workqueue(card->workqueue);
    while (card->packets) {
        packet = card->packets;
        card->packets = card->packets->next;
        kfree(packet);
    }
    // SSV --
    #if 0
    if (card->helper_allocated)
        kfree(card->helper);
    #endif
    if (card->firmware_allocated)
        kfree(card->firmware);
    kfree(card);

    goto out;
}

static void cabrio_sdio_remove(struct sdio_func *func)
{
    struct cabrio_sdio_card *card;
    struct cabrio_sdio_packet *packet;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    card = sdio_get_drvdata(func);

    /* Undo decrement done above in if_sdio_probe */
    pm_runtime_get_noresume(&func->dev);
    //SSV ??
    #if 0
    if (user_rmmod && (card->model == MODEL_8688)) {
        /*
         * FUNC_SHUTDOWN is required for SD8688 WLAN/BT
         * multiple functions
         */
        struct cmd_header cmd;

        memset(&cmd, 0, sizeof(cmd));

        cabrio_dbg_sdio("send function SHUTDOWN command\n");
        if (__cabrio_cmd(card->priv, CMD_FUNC_SHUTDOWN,
                &cmd, sizeof(cmd), cabrio_cmd_copyback,
                (unsigned long) &cmd))
            pr_alert("CMD_FUNC_SHUTDOWN cmd failed\n");
    }
    #endif

    cabrio_dbg_sdio("call remove card\n");
    cabrio_stop_card(card->priv);
    cabrio_remove_card(card->priv);

    flush_workqueue(card->workqueue);
    destroy_workqueue(card->workqueue);

    while (card->packets) {
        packet = card->packets;
        card->packets = card->packets->next;
        kfree(packet);
    }
    // SSV --
    #if 0
    if (card->helper_allocated)
        kfree(card->helper);
    #endif 
    if (card->firmware_allocated)
        kfree(card->firmware);
    kfree(card);

    cabrio_dbg_leave(CABRIO_DBG_SDIO);
}

static int cabrio_sdio_suspend(struct device *dev)
{
    struct sdio_func *func = dev_to_sdio_func(dev);
    int ret;
    struct cabrio_sdio_card *card = sdio_get_drvdata(func);

    mmc_pm_flag_t flags = sdio_get_host_pm_caps(func);

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    dev_info(dev, "%s: suspend: PM flags = 0x%x\n",
         sdio_func_id(func), flags);

    /* If we aren't being asked to wake on anything, we should bail out
     * and let the SD stack power down the card.
     */
    if (card->priv->wol_criteria == EHS_REMOVE_WAKEUP) {
        dev_info(dev, "Suspend without wake params -- powering down card\n");
        ret = -ENOSYS;
        goto out;
    }

    if (!(flags & MMC_PM_KEEP_POWER)) {
        dev_err(dev, "%s: cannot remain alive while host is suspended\n",
            sdio_func_id(func));
        ret = -ENOSYS;
        goto out;
    }

    ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
    if (ret)
        goto out;

    ret = cabrio_suspend(card->priv);
    if (ret)
        goto out;

    ret = sdio_set_host_pm_flags(func, MMC_PM_WAKE_SDIO_IRQ);
out:
    cabrio_dbg_leave(CABRIO_DBG_SDIO);
    return ret;
}

static int cabrio_sdio_resume(struct device *dev)
{
    struct sdio_func *func = dev_to_sdio_func(dev);
    struct cabrio_sdio_card *card = sdio_get_drvdata(func);
    int ret;

    dev_info(dev, "%s: resume: we're back\n", sdio_func_id(func));

    ret = cabrio_resume(card->priv);

    return ret;
}

static const struct dev_pm_ops cabrio_sdio_pm_ops = {
    .suspend    = cabrio_sdio_suspend,
    .resume        = cabrio_sdio_resume,
};

static struct sdio_driver cabrio_sdio_driver = {
    .name        = "cabrio_fmac_sdio",
    .id_table    = cabrio_sdio_ids,
    .probe        = cabrio_sdio_probe,
    .remove        = cabrio_sdio_remove,
    .drv = {
        .pm = &cabrio_sdio_pm_ops,
    },
};




/*******************************************************************/
/* Module functions                                                */
/*******************************************************************/

static int __init cabrio_sdio_init_module(void)
{
    int ret = 0;

    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    printk(KERN_INFO "cabrio_sdio: SSV6200 Cabrio SDIO driver\n");
    printk(KERN_INFO "cabrio_sdio: Copyright South Silicon Valley Inc.\n");

    ret = sdio_register_driver(&cabrio_sdio_driver);

    /* Clear the flag in case user removes the card. */
    user_rmmod = 0;

    cabrio_dbg_leave_args(CABRIO_DBG_SDIO, "ret %d", ret);

    return ret;
}

static void __exit cabrio_sdio_exit_module(void)
{
    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    /* Set the flag as user is removing this module. */
    user_rmmod = 1;

    cancel_work_sync(&card_reset_work);

    sdio_unregister_driver(&cabrio_sdio_driver);

    cabrio_dbg_leave(CABRIO_DBG_SDIO);
}

module_init(cabrio_sdio_init_module);
module_exit(cabrio_sdio_exit_module);

MODULE_DESCRIPTION("SSV6200 SDIO WLAN Driver");
MODULE_AUTHOR("South Silicon Valley Inc.");
MODULE_LICENSE("GPL");

