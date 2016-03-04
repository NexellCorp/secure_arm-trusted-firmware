/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __NX_UART_H__
#define __NX_UART_H__

/* NX_UART Registers */
#define UARTDLCON		0x00 /* Line Control */
#define UARTRUCON		0x04 /* Control */
#define UARTEFCON		0x08 /* FIFO Control */
#define UARTFMCON		0x0C /* Modem Control */
#define UARTUSTATUS		0x10 /* Tx/Rx Status */
#define UARTESTATUS		0x14 /* Rx Error Status */
#define UARTFSTATUS		0x18 /* FIFO Status */
#define UARTMSTATUS		0x1C /* Modem Status */
#define UARTTXH			0x20 /* Transmit Buffer */
#define UARTRXH			0x24 /* Receive Buffer */
#define UARTBRDR		0x28 /* Baud Rate Driver */
#define UARTFRACVAL		0x2C /* Driver Fractional Value */
#define UARTINTP		0x30 /* Instrrupt Pending */
#define UARTINTSP		0x34 /* Instrrupt Source */
#define UARTINTM		0x38 /* Instrrupt Mask */

/* Data status bits */
#define UART_DATA_ERROR_MASK	0x0F00

/* Status reg bits */
#define UART_STATUS_ERROR_MASK	0x0F

/* Flag reg bits */
#define NX_UARTFR_RI		(1 << 8) /* Ring indicator */
#define NX_UARTFR_TXFE		(1 << 7) /* Transmit FIFO empty */
#define NX_UARTFR_RXFF		(1 << 6) /* Receive  FIFO full */
#define NX_UARTFR_TXFF		(1 << 5) /* Transmit FIFO full */
#define NX_UARTFR_RXFE		(1 << 4) /* Receive  FIFO empty */
#define NX_UARTFR_BUSY		(1 << 3) /* UART busy */
#define NX_UARTFR_DCD		(1 << 2) /* Data carrier detect */
#define NX_UARTFR_DSR		(1 << 1) /* Data set ready */
#define NX_UARTFR_CTS		(1 << 0) /* Clear to send */

#define NX_UARTFSTATUS_TXFF_BIT 24
				/* Transmit FIFO full bit in UARTFR register */
#define NX_UARTFSTATUS_RXFE_BIT 4
				/* Receive FIFO empty bit in UARTFR register */

/* Control reg bits */
#define NX_UARTCR_CTSEN		(1 << 15) /* CTS hardware flow control enable */
#define NX_UARTCR_RTSEN		(1 << 14) /* RTS hardware flow control enable */
#define NX_UARTCR_RTS		(1 << 11) /* Request to send */
#define NX_UARTCR_DTR		(1 << 10) /* Data transmit ready. */
#define NX_UARTCR_RXE		(1 << 9)  /* Receive enable */
#define NX_UARTCR_TXE		(1 << 8)  /* Transmit enable */
#define NX_UARTCR_LBE		(1 << 7)  /* Loopback enable */
#define NX_UARTCR_UARTEN	(1 << 0)  /* UART Enable */

#if !defined(NX_LINE_CONTROL)
		/* FIFO Enabled / No Parity / 8 Data bit / One Stop Bit */
#define NX_LINE_CONTROL (NX_UARTLCR_H_FEN | NX_UARTLCR_H_WLEN_8)
#endif

/* Line Control Register Bits */
#define NX_UARTLCR_H_SPS	(1 << 7) /* Stick parity select */
#define NX_UARTLCR_H_WLEN_8	(3 << 5)
#define NX_UARTLCR_H_WLEN_7	(2 << 5)
#define NX_UARTLCR_H_WLEN_6	(1 << 5)
#define NX_UARTLCR_H_WLEN_5	(0 << 5)
#define NX_UARTLCR_H_FEN	(1 << 4)  /* FIFOs Enable */
#define NX_UARTLCR_H_STP2	(1 << 3)  /* Two stop bits select */
#define NX_UARTLCR_H_EPS	(1 << 2)  /* Even parity select */
#define NX_UARTLCR_H_PEN	(1 << 1)  /* Parity Enable */
#define NX_UARTLCR_H_BRK	(1 << 0)  /* Send break */

#endif /* __NX_UART_H__ */
