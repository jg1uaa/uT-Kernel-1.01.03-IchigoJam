/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 2.0 Software Package
 *
 *    Copyright (C) 2006-2014 by Ken Sakamura.
 *    This software is distributed under the T-License 2.0.
 *----------------------------------------------------------------------
 *
 *    Released by T-Engine Forum(http://www.t-engine.org/) at 2014/09/01.
 *
 *----------------------------------------------------------------------
 */

/*
 *	tkdev_conf.h (LPC1114)
 *	Target System Configuration
 */

#ifndef _TKDEV_CONF_
#define _TKDEV_CONF_
/* Also included from assembler source */

#define ARM_INT_MASK 0xC0
#define ARM_IRQ_BIT  0x80
#define ARM_FIQ_BIT  0x40
#define ARM_THM_BIT  0x20


// swkim_timer
#define TMCLK		(CPU_CLOCK / 1000000)	// MHz

/*
 * Timer interrupt level
 */
#define TIMER_INTLEVEL		0

#endif /* _TKDEV_CONF_ */
