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
 *	@(#)disint.S (libtk/Cortex-M0)
 *
 *	Enable/disable external interrupt
 */

#include <machine.h>
#include <tk/sysdef.h>

	.code 16
	.syntax unified
	.thumb
/*
 * Disable external interrupt
 * UINT disint( void )
 *	Disable interrupt by Rise prioperty to 0.
 */
	.text
	.align	2
	.thumb_func
	.globl	Csym(disint)
	.type	Csym(disint), %function
Csym(disint):
	mrs	r0, primask
	cpsid	i
	bx	lr


/*
 * Enable external interrupt (restore to original state)
 * UINT enaint( UINT intsts )
 *	Enable interrupt.
 */
	.text
	.align	2
	.thumb_func
	.globl	Csym(enaint)
	.type	Csym(enaint), %function
Csym(enaint):
	msr	primask, r0
	bx	lr

	.end
