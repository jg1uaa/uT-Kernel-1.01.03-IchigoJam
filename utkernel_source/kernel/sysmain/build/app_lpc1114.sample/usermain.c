/*
 *	usermain.c (usermain)
 *	User Main
 */

#include <basic.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "utk_config.h"

#define	IOCON_PIO1_5	((_UW *)0x400440a0)
#define	GPIO1DIR	((_UW *)0x50018000)
#define	GPIO1DATA_5	((_UW *)0x50010080)

#define	LED_EVENT	(1 << 0)
#define	UART_EVENT	(1 << 1)

#define	SYSAHBCLKCTRL	((_UW *)0x40048080)

#define	TimerBase	0x4000c000	/* CT16B0 */
#define	TnIR		((_UW *)(TimerBase + 0x0000))
#define	TnTCR		((_UW *)(TimerBase + 0x0004))
#define	TnTC		((_UW *)(TimerBase + 0x0008))
#define	TnPR		((_UW *)(TimerBase + 0x000c))
#define	TnPC		((_UW *)(TimerBase + 0x0010))
#define	TnMCR		((_UW *)(TimerBase + 0x0014))
#define	TnMR0		((_UW *)(TimerBase + 0x0018))
#define	TnMR1		((_UW *)(TimerBase + 0x001c))
#define	TnMR2		((_UW *)(TimerBase + 0x0020))
#define	TnMR3		((_UW *)(TimerBase + 0x0024))
#define	TnCCR		((_UW *)(TimerBase + 0x0028))
#define	TnCR0		((_UW *)(TimerBase + 0x002c))
#define	TnEMR		((_UW *)(TimerBase + 0x003c))
#define	TnCTCR		((_UW *)(TimerBase + 0x0070))
#define	TnPWMC		((_UW *)(TimerBase + 0x0074))

#define	TimerVec	16
#define	TnMR0Val	0x0800

LOCAL	ID	FlgID;
LOCAL	ID	TskID;
LOCAL	UW	Counter = 0;

LOCAL	void	handler(void)
{
	*TnIR = 0x1f;

	tk_set_flg(FlgID, LED_EVENT);

	return;
}

LOCAL	void	task(void)
{
	UW	imask;
	ER	er;
	UINT	flgptn;
	LOCAL	const UB	d[] = "-\\|/";

	while (1) {
		er = tk_wai_flg(FlgID, ~0, TWF_ORW | TWF_BITCLR,
				&flgptn, TMO_FEVR);
		if (er < E_OK) goto fin0;

		if (flgptn & UART_EVENT) {
			tm_putchar(d[Counter++ & 3]);
			tm_putchar('\r');
		}
		if (flgptn & LED_EVENT) {
			DI(imask);
			*GPIO1DATA_5 ^= 0x020;
			EI(imask);
		}
	}

fin0:
	tk_exd_tsk();
}

EXPORT	INT	usermain(void)
{
	UW	imask;
	ER	er;
	T_DINT	dint = {
		.intatr = TA_HLNG,
		.inthdr = handler,
	};
	T_CFLG	cflg = {
		.exinf = NULL,
		.flgatr = TA_TFIFO | TA_WSGL,
		.iflgptn = 0,
	};
	T_CTSK	ctsk = {
		.exinf = NULL,
		.tskatr = TA_HLNG | TA_RNG0,
		.task = task,
		.itskpri = INIT_TASK_PRI,
		.stksz = 512,	// task() required about 160 bytes stack
	};

	/* GPIO setup */
	DI(imask);
	*IOCON_PIO1_5 = 0xd0;	// (default)
	*GPIO1DIR = 0x20;	// select output
	*GPIO1DATA_5 = 0x00;	// turn off
	EI(imask);

	/* event flag setup */
	er = tk_cre_flg(&cflg);
	if (er < E_OK) goto fin0;
	FlgID = er;

	/* interrupt handler setup */
	er = tk_def_int(TimerVec, &dint);
	if (er < E_OK) goto fin1;

	/* Timer setup */
	DI(imask);
	*SYSAHBCLKCTRL |= 0x80;	// power-up timer
	*TnTCR = 0;		// stop timer
	*TnCTCR = 0;		// timer mode
	*TnEMR = 0;		// external match: do nothing
	*TnPWMC = 0;		// PWM disable
	*TnCCR = 0;		// capture control: all disable
	*TnPR = 614;		// prescaler: PCLK/615
	*TnMR0 = TnMR0Val;
	*TnPC = 0;		// reset prescale count
	*TnTC = 0;		// reset timer count
	*TnIR = 0x1f;		// clear all interrupts
	*TnMCR = 0x03;		// interrupt enable, reset on MR
	*TnTCR = 1;		// start timer
	EI(imask);
	EnableInt(TimerVec, 0x40);	// pri = 1 (same as systick)

	/* task setup */
	er = tk_cre_tsk(&ctsk);
	if (er < E_OK) goto fin2;
	TskID = er;

	er = tk_sta_tsk(TskID, 0);
	if (er < E_OK) goto fin3;

	/* task */
	while (1) {
		tk_set_flg(FlgID, UART_EVENT);
		tk_dly_tsk(500);
	}

/*fin4:*/
	tk_ter_tsk(TskID);
fin3:
	tk_del_tsk(TskID);
fin2:
	DisableInt(TimerVec);
	*TnTCR = 0;
	tk_def_int(TimerVec, NULL);
fin1:
	tk_del_flg(FlgID);
fin0:
	tm_putstring((UB*)"aborted\n");

	return 0;
}
