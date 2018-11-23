// original code by Takayoshi SASANO
// this code is licensed under NYSL 0.9982, http://www.kmonos.net/nysl/NYSL.TXT

#include <basic.h>

#define	uartBase		0x40008000	// UART0
#define	uartStep		0x04
#define	uartClock		(CPU_CLOCK / 1)	// Hz
#define	uartSpeed		38400		// bps
#define	uartDivisor		(uartClock / (uartSpeed * 16))
Inline	UW	uartRead(UW adr) {return *(_UW*)adr;}
Inline	void	uartWrite(UW adr, UW dat) {*(_UW*)adr = dat;}

#define	RHR		(uartBase + (0x00 * uartStep))	// r
#define	THR		(uartBase + (0x00 * uartStep))	//  w
#define	IER		(uartBase + (0x01 * uartStep))	// rw
#define	ISR		(uartBase + (0x02 * uartStep))	// r
#define	FCR		(uartBase + (0x02 * uartStep))	//  w
#define	LCR		(uartBase + (0x03 * uartStep))	// rw
#define	MCR		(uartBase + (0x04 * uartStep))	// rw
#define	LSR		(uartBase + (0x05 * uartStep))	// r

#define	DLL		(uartBase + (0x00 * uartStep))	// rw
#define	DLM		(uartBase + (0x01 * uartStep))	// rw

/* LPC111X specific: pin selection, power control */
#define	IOCON_PIO1_6	((_UW *)0x400440a4)
#define	IOCON_PIO1_7	((_UW *)0x400440a8)
#define	AHBCLKCTRL	((_UW *)0x40048080)
#define	UARTCLKDIV	((_UW *)0x40048098)


int	sio_send_frame(const unsigned char *buf, int size)
{
	int	i;

	for (i = 0; i < size; i++) {
		while(!(uartRead(LSR) & 0x20));		// wait for Tx ready
		uartWrite(THR, *buf++);
	}

	return i;
}

int	sio_recv_frame(unsigned char* buf, int size)
{
	int	i, s, c;

	for (i = 0; i < size; i++) {
		while (1) {
			s = uartRead(LSR);		// get status
			if (s & 0x1f) {			// Rx ready
				c = uartRead(RHR);	// read from FIFO
				if (!(s & 0x1e)) {	// no error
					*buf++ = c;
					break;
				}
			}
		}
	}

	return i;
}

void	sio_init(void)
{
	*IOCON_PIO1_6 = 0xc1;		// enable RXD pin
	*IOCON_PIO1_7 = 0xc1;		// enable TXD pin
	*UARTCLKDIV = 0x01;		// UART_PCLK = system clock / 1
	*AHBCLKCTRL |= (1 << 12);	// UART power-up

	uartWrite(IER, 0x00);		// disable interrupt
	uartWrite(LCR, 0x80);		// set divisor
	uartWrite(DLM, 0xff);		// (avoid divisor=0)
	uartWrite(DLL, (uartDivisor >> 0) & 0xff);
	uartWrite(DLM, (uartDivisor >> 8) & 0xff);
	uartWrite(LCR, 0x03);		// data 8bit, stop 1bit, non-parity
	uartWrite(MCR, 0x03);		// RTS#, DTR# assert
	uartWrite(FCR, 0x07);		// FIFO enable and reset

	return;
}
