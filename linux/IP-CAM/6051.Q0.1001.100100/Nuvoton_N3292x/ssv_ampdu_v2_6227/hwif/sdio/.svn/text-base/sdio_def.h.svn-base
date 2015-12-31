#ifndef _SDIO_DEF_H_
#define _SDIO_DEF_H_

#include <linux/scatterlist.h>


/* -------------- h/w register ------------------- */
#define BASE_SDIO	0

/* Note :
	For now, the reg of SDIO Host & Card Controller are the same.
If it changes in the future, you should define again.
*/
#define REG_DATA_IO_PORT_0		(BASE_SDIO + 0x00)		// 0
#define REG_DATA_IO_PORT_1	    (BASE_SDIO + 0x01)		// 0
#define REG_DATA_IO_PORT_2		(BASE_SDIO + 0x02)		// 0
#define REG_INT_MASK			(BASE_SDIO + 0x04)		// 0
#define REG_INT_STATUS			(BASE_SDIO + 0x08)		// 0
#define REG_INT_TRIGGER			(BASE_SDIO + 0x09)		// 0
#define REG_Fn1_STATUS			(BASE_SDIO + 0x0c)		// 0
#define REG_CARD_PKT_LEN_0		(BASE_SDIO + 0x10)		// 0
#define REG_CARD_PKT_LEN_1		(BASE_SDIO + 0x11)		// 0
#define REG_CARD_FW_DL_STATUS	(BASE_SDIO + 0x12)		// 0
#define REG_CARD_SELF_TEST		(BASE_SDIO + 0x13)		// 0
#define REG_CARD_RCA_0			(BASE_SDIO + 0x20)		// 0
#define REG_CARD_RCA_1			(BASE_SDIO + 0x21)		// 0
#define REG_SDIO_FIFO_WR_THLD_0	(BASE_SDIO + 0x24)		// 80
#define REG_SDIO_FIFO_WR_THLD_1	(BASE_SDIO + 0x25)		// 0
#define REG_OUTPUT_TIMING_REG	(BASE_SDIO + 0x55)		// 0
//#define REG_PMU_WAKEUP			(BASE_SDIO + 0x64)		// 0 FPGA
#define REG_PMU_WAKEUP			(BASE_SDIO + 0x67)		// 0
#define REG_REG_IO_PORT_0		(BASE_SDIO + 0x70)		// 0
#define REG_REG_IO_PORT_1	    (BASE_SDIO + 0x71)		// 0
#define REG_REG_IO_PORT_2		(BASE_SDIO + 0x72)		// 0

//SDIO TX ALLOCATE FUNCTION
#define REG_SDIO_TX_ALLOC_SIZE	(BASE_SDIO + 0x98)		// 0
#define REG_SDIO_TX_ALLOC_SHIFT	(BASE_SDIO + 0x99)		// 0
#define REG_SDIO_TX_ALLOC_STATE	(BASE_SDIO + 0x9a)		// 0

#define REG_SDIO_TX_INFORM_0	(BASE_SDIO + 0x9c)		// 0
#define REG_SDIO_TX_INFORM_1	(BASE_SDIO + 0x9d)		// 0
#define REG_SDIO_TX_INFORM_2	(BASE_SDIO + 0x9e)		// 0

#define SDIO_TX_ALLOC_SUCCESS	0x01
#define SDIO_TX_NO_ALLOC		0x02
#define SDIO_TX_DULPICATE_ALLOC	0x04
#define SDIO_TX_TX_DONE     	0x08
#define SDIO_TX_AHB_HANG     	0x10
#define SDIO_TX_MB_FULL     	0x80

#define SDIO_HCI_IN_QUEUE_EMPTY	0x04
#define SDIO_EDCA0_SHIFT		4

#define SDIO_TX_ALLOC_SIZE_SHIFT	0x07
#define SDIO_TX_ALLOC_ENABLE		0x10

/* -------------- default      ------------------- */
#define SDIO_DEF_BLOCK_SIZE			0x80		// should be the multiple of 8 bytes 
#if (SDIO_DEF_BLOCK_SIZE % 8)
#error Wrong SDIO_DEF_BLOCK_SIZE value!! Should be the multiple of 8 bytes!!!!!!!!!!!!!!!!!!!!!!
#endif

//output timing
// 0: cmd  [0]:positive [1]:negative
// 1: data [0]:positive [1]:negative
#ifdef CONFIG_SSV_SUPPORT_AW_SUNXI
#define SDIO_DEF_OUTPUT_TIMING 		3
#else
#define SDIO_DEF_OUTPUT_TIMING 		0
#endif
// block mode threshold
#define SDIO_DEF_BLOCK_MODE_THRD	128
#if (SDIO_DEF_BLOCK_MODE_THRD % 8)
#error Wrong SDIO_DEF_BLOCK_MODE_THRD value!! Should be the multiple of 8 bytes!!!!!!!!!!!!!!!!!!!!!!
#endif

// 0: false, 1: true
#define SDIO_DEF_FORCE_BLOCK_MODE	0


/* Scatter/Gather related */
#define MAX_SCATTER_ENTRIES_PER_REQ          8


struct sdio_scatter_item {
	u8 *buf;
	int len;
};

struct sdio_scatter_req {

	/* request flags */
	u32 req;

	/* total length of entire transfer */
	u32 len;

	int scat_entries;

	/* bounce buffer for upper layers to copy to/from */
	struct sdio_scatter_item scat_list[MAX_SCATTER_ENTRIES_PER_REQ];

	struct scatterlist sgentries[MAX_SCATTER_ENTRIES_PER_REQ];
    
};

//-------------------------------------------------------------
/* request flags of  sdio_scatter_req*/
/* direction of transfer (read/write) */
#define SDIO_READ                    0x00000001
#define SDIO_WRITE                   0x00000002


//cmd 53 r/w flags
#define CMD53_ARG_READ          0
#define CMD53_ARG_WRITE         1

//cmd53 block basis
#define CMD53_ARG_BLOCK_BASIS   1

//cmd 53 opcode
#define CMD53_ARG_FIXED_ADDRESS 0
#define CMD53_ARG_INCR_ADDRESS  1


#ifdef  CONFIG_FW_ALIGNMENT_CHECK
#define SDIO_DMA_BUFFER_LEN                 2048 
#endif


//-------------------------------------------------------------



#endif /* _SDIO_DEF_H_ */
