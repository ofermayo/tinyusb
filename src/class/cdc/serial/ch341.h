#ifndef TUSB_CH341_H
#define TUSB_CH341_H

#define TU_CH341_VID_PID_PAIRS \
   0x1A86, 0x5523, \
   0x1A86, 0x7522, \
   0x1A86, 0x7523, \
   0x2184, 0x0057, \
   0x4348, 0x5523, \
   0x9986, 0x7523

#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_TIMEOUT   1000

/* flags for IO-Bits */
#define CH341_BIT_RTS (1 << 6)
#define CH341_BIT_DTR (1 << 5)

/******************************/
/* interrupt pipe definitions */
/******************************/
/* always 4 interrupt bytes */
/* first irq byte normally 0x08 */
/* second irq byte base 0x7d + below */
/* third irq byte base 0x94 + below */
/* fourth irq byte normally 0xee */

/* second interrupt byte */
#define CH341_MULT_STAT 0x04 /* multiple status since last interrupt event */

/* status returned in third interrupt answer byte, inverted in data
   from irq */
#define CH341_BIT_CTS 0x01
#define CH341_BIT_DSR 0x02
#define CH341_BIT_RI  0x04
#define CH341_BIT_DCD 0x08
#define CH341_BITS_MODEM_STAT 0x0f /* all bits */

/* Break support - the information used to implement this was gleaned from
 * the Net/FreeBSD uchcom.c driver by Takanori Watanabe.  Domo arigato.
 */

#define CH341_REQ_READ_VERSION 0x5F
#define CH341_REQ_WRITE_REG    0x9A
#define CH341_REQ_READ_REG     0x95
#define CH341_REQ_SERIAL_INIT  0xA1
#define CH341_REQ_MODEM_CTRL   0xA4

#define CH341_REG_BREAK        0x05
#define CH341_REG_PRESCALER    0x12
#define CH341_REG_DIVISOR      0x13
#define CH341_REG_LCR          0x18
#define CH341_REG_LCR2         0x25

#define CH341_NBREAK_BITS      0x01

#define CH341_LCR_ENABLE_RX    0x80
#define CH341_LCR_ENABLE_TX    0x40
#define CH341_LCR_MARK_SPACE   0x20
#define CH341_LCR_PAR_EVEN     0x10
#define CH341_LCR_ENABLE_PAR   0x08
#define CH341_LCR_STOP_BITS_2  0x04
#define CH341_LCR_CS8          0x03
#define CH341_LCR_CS7          0x02
#define CH341_LCR_CS6          0x01
#define CH341_LCR_CS5          0x00

#define CH341_QUIRK_LIMITED_PRESCALER	1
#define CH341_QUIRK_SIMULATE_BREAK	1 << 1

#define CH341_CLKRATE		48000000
#define CH341_CLK_DIV(ps, fact)	(1 << (12 - 3 * (ps) - (fact)))
#define CH341_MIN_RATE(ps)	(CH341_CLKRATE / (CH341_CLK_DIV((ps), 1) * 512))

const uint32_t ch341_min_rates[] = {
	CH341_MIN_RATE(0),
	CH341_MIN_RATE(1),
	CH341_MIN_RATE(2),
	CH341_MIN_RATE(3),
};

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

/* Supported range is 46 to 3000000 bps. */
#define CH341_MIN_BPS	DIV_ROUND_UP(CH341_CLKRATE, CH341_CLK_DIV(0, 0) * 256)
#define CH341_MAX_BPS	(CH341_CLKRATE / (CH341_CLK_DIV(3, 0) * 2))

#endif