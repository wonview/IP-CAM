#ifndef _SSV_SDIO_H_
#define _SSV_SDIO_H_

/* -------------- h/w register ------------------- */
#define BASE_SDIO	0

/* Note :
	For now, the reg of SDIO Host & Card Controller are the same.
If it changes in the future, you should define again.
*/
#define REG_IO_PORT_0			(BASE_SDIO + 0x00)		// 0
#define REG_IO_PORT_1			(BASE_SDIO + 0x01)		// 0
#define REG_IO_PORT_2			(BASE_SDIO + 0x02)		// 0
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
//SDIO TX ALLOCATE FUNCTION
#define REG_SDIO_TX_ALLOC_SIZE	(BASE_SDIO + 0x98)		// 0
#define REG_SDIO_TX_ALLOC_SHIFT	(BASE_SDIO + 0x99)		// 0
#define REG_SDIO_TX_ALLOC_STATE	(BASE_SDIO + 0x9a)		// 0

#define SDIO_TX_ALLOC_SUCCESS	0x01
#define SDIO_TX_NO_ALLOC		0x02
#define SDIO_TX_DULPICATE_ALLOC	0x04
#define SDIO_TX_MAILBOX_FULL    0x80

#define SDIO_TX_ALLOC_SIZE_SHIFT	0x04
#define SDIO_TX_ALLOC_ENABLE		0x10

#define SDIO_RX_READY_INT       (0x01)
#define SDIO_TX_DONE_INT        (0x02)

/* -------------- default      ------------------- */
//Option[50MHz 25MHz 12.5MHz 6.25MHz] 
#ifdef SDIO_TEST
#define SDIO_DEF_CLOCK_RATE			12500		// 25MHz
#else
#define SDIO_DEF_CLOCK_RATE			12500		// 25MHz
#endif
#define SDIO_DEVICE_NAME			"\\\\.\\SSVCabrio"
#define SDIO_DEF_BLOCK_SIZE			0x80		// should be the multiple of 8 bytes 
#if (SDIO_DEF_BLOCK_SIZE % 8)
#error Wrong SDIO_DEF_BLOCK_SIZE value!! Should be the multiple of 8 bytes!!!!!!!!!!!!!!!!!!!!!!
#endif

//output timing
// 0: cmd  [0]:positive [1]:negative
// 1: data [0]:positive [1]:negative
#define SDIO_DEF_OUTPUT_TIMING 		3

// block mode threshold
#define SDIO_DEF_BLOCK_MODE_THRD	128
#if (SDIO_DEF_BLOCK_MODE_THRD % 8)
#error Wrong SDIO_DEF_BLOCK_MODE_THRD value!! Should be the multiple of 8 bytes!!!!!!!!!!!!!!!!!!!!!!
#endif

// 0: false, 1: true
#define SDIO_DEF_FORCE_BLOCK_MODE	0

#endif /* _SSV_SDIO_H_ */

