/*
 * sx1232_reg.h
 *
 *  Created on: 20 jun. 2018
 *      Author: Ludo
 */

#ifndef __SX1232_REG_H__
#define __SX1232_REG_H__

/*** SX1232 registers map ***/

#define SX1232_REG_FIFO				0x00
#define SX1232_REG_OPMODE			0x01
#define SX1232_REG_BITRATEMSB		0x02
#define SX1232_REG_BITRATELSB		0x03
#define SX1232_REG_FDEVMSB			0x04
#define SX1232_REG_FDEVLSB			0x05
#define SX1232_REG_FRFMSB			0x06
#define SX1232_REG_FRFMID			0x07
#define SX1232_REG_FRFLSB			0x08
#define SX1232_REG_PACONFIG			0x09
#define SX1232_REG_PARAMP			0x0A
#define SX1232_REG_OCP				0x0B
#define SX1232_REG_LNA				0x0C
#define SX1232_REG_RXCONFIG			0x0D
#define SX1232_REG_RSSICONFIG		0x0E
#define SX1232_REG_RSSICOLLISION	0x0F
#define SX1232_REG_RSSITHRESH		0x10
#define SX1232_REG_RSSIVALUE		0x11
#define SX1232_REG_RXBW				0x12
#define SX1232_REG_AFCBW			0x13
#define SX1232_REG_OOKPEAK			0x14
#define SX1232_REG_OOKFIX			0x15
#define SX1232_REG_OOKAVG			0x16
// 0x17-0x19 = reserved.
#define SX1232_REG_AFCFEI			0x1A
#define SX1232_REG_AFCMSB			0x1B
#define SX1232_REG_AFCLSB			0x1C
#define SX1232_REG_FEIMSB			0x1D
#define SX1232_REG_FEILSB			0x1E
#define SX1232_REG_PREAMBLEDETECT	0x1F
#define SX1232_REG_RXTIMEOUT1		0x20
#define SX1232_REG_RXTIMEOUT2		0x21
#define SX1232_REG_RXTIMEOUT3		0x22
#define SX1232_REG_RXDELAY			0x23
#define SX1232_REG_OSC				0x24
#define SX1232_REG_PREAMBLEMSB		0x25
#define SX1232_REG_PREAMBLELSB		0x26
#define SX1232_REG_SYNCCONFIG		0x27
#define SX1232_REG_SYNCVALUE1		0x28
#define SX1232_REG_SYNCVALUE2		0x29
#define SX1232_REG_SYNCVALUE3		0x2A
#define SX1232_REG_SYNCVALUE4		0x2B
#define SX1232_REG_SYNCVALUE5		0x2C
#define SX1232_REG_SYNCVALUE6		0x2D
#define SX1232_REG_SYNCVALUE7		0x2E
#define SX1232_REG_SYNCVALUE8		0x2F
#define SX1232_REG_PACKETCONFIG1	0x30
#define SX1232_REG_PACKETCONFIG2	0x31
#define SX1232_REG_PAYLOADLENGTH	0x32
#define SX1232_REG_NODEADRS			0x33
#define SX1232_REG_BROADCASTADRS	0x34
#define SX1232_REG_FIFOTHRESH		0x35
#define SX1232_REG_SEQCONFIG1		0x36
#define SX1232_REG_SEQCONFIG2		0x37
#define SX1232_REG_TIMERRESOL		0x38
#define SX1232_REG_TIMER1COEF		0x39
#define SX1232_REG_TIMER2COEF		0x3A
#define SX1232_REG_IMAGECAL			0x3B
#define SX1232_REG_TEMP				0x3C
#define SX1232_REG_LOWBAT			0x3D
#define SX1232_REG_IRQFLAGS1		0x3E
#define SX1232_REG_IRQFLAGS2		0x3F
#define SX1232_REG_DIOMAPPING1		0x40
#define SX1232_REG_DIOMAPPING2		0x41
#define SX1232_REG_VERSION			0x42
#define SX1232_REG_AGCREF			0x43
#define SX1232_REG_AGCTHRESH1		0x44
#define SX1232_REG_AGCTHRESH2		0x45
#define SX1232_REG_AGCTHRESH3		0x46
// 0x47-0x4A = reserved.
#define SX1232_REG_PLLHOP			0x4B
#define SX1232_REG_PAVALUE			0x4C
// 0x4C-0x57 = reserved
#define SX1232_REG_TCXO				0x58
// 0x59 = reserved.
#define SX1232_REG_PADAC			0x5A
// 0x5B = reserved.
#define SX1232_REG_PLL				0x5C
// 0x5D = reserved.
#define SX1232_REG_PLLLOWPN			0x5E
// 0x5F-0x6B = reserved.
#define SX1232_REG_PAMANUAL			0x63
#define SX1232_REG_FORMERTEMP		0x6C
// 0x6D-0x6F = reserved.
#define SX1232_REG_BITRATEFRAC		0x70
// Last register address
#define SX1232_REG_LAST				0x7F

#endif /* __SX1232_REG_H__ */
