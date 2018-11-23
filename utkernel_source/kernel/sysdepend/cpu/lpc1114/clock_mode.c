// original code by Takayoshi SASANO
// this code is licensed under NYSL 0.9982, http://www.kmonos.net/nysl/NYSL.TXT

#include <tk/tkernel.h>

#include "hwconfig.h"
#include "utk_config.h"

#include <machine.h>


/*
 * Clock Settings:
 * ---------------
 *	IRC oscillator: 12MHz
 *	Fcco:           192MHz
 *	FCLKOUT:        48MHz
 */

#define	SYSCONBase	0x40048000
#define	SYSMEMREMAP	((_UW *)(SYSCONBase + 0x000))
#define	SYSPLLCTRL	((_UW *)(SYSCONBase + 0x008))
#define	SYSPLLSTAT	((_UW *)(SYSCONBase + 0x00c))
#define	SYSPLLCLKSEL	((_UW *)(SYSCONBase + 0x040))
#define	SYSPLLCLKUEN	((_UW *)(SYSCONBase + 0x044))
#define	MAINCLKSEL	((_UW *)(SYSCONBase + 0x070))
#define	MAINCLKUEN	((_UW *)(SYSCONBase + 0x074))
#define	AHBCLKCTRL	((_UW *)(SYSCONBase + 0x080))
#define	PDRUNCFG	((_UW *)(SYSCONBase + 0x238))

EXPORT void init_clock_control(void)
{
	/* User Flash Mode, disable interrupt vector remap */
	*SYSMEMREMAP = 0x02;

	/* main clock: IRC oscillator */
	*MAINCLKSEL = 0x00;
	*MAINCLKUEN = 0x00;
	*MAINCLKUEN = 0x01;

	/* System PLL: power-up */
	*PDRUNCFG &= ~0x80;

	/* PLL clock source: IRC oscillator */
	*SYSPLLCLKSEL = 0x00;
	*SYSPLLCLKUEN = 0x00;
	*SYSPLLCLKUEN = 0x01;

	/*
	 * Fcco = 2 x P x FCLKOUT
	 *   192MHz = 2 x 2 x 48MHz
	 * M = FCLKOUT / FCLKIN
	 *   4 = 48MHz / 12MHz
	 */
	*SYSPLLCTRL = 0x23;

	/* wait for PLL is lock */
	while (!(*SYSPLLSTAT & 0x01));

	/* main clock: system PLL */
	*MAINCLKSEL = 0x03;
	*MAINCLKUEN = 0x00;
	*MAINCLKUEN = 0x01;

	/* enable IOCON clock */
	*AHBCLKCTRL |= 0x00010000;

	return;
}
