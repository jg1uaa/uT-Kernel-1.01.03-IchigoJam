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
 *  @(#) vector.S
 *
 *    Vector table and interrupt handlers
 *
 */

#include "hwconfig.h"
#include "utk_config.h"

#include <machine.h>
#include <tk/asm.h>

	.code 16
	.syntax unified
	.thumb

/*
 * Vector Table
 */
	.section .vector
	.align 2

__vector:
	.word	EXCEPTION_STACK_TOP 		/* 0 -> sp value for reset */
	.word	Csym(Reset_Handler)		/* 1 -> reset */
	.word	Csym(nmi_handler)		/* 2 -> Non maskable interrupt */
	.word	Csym(hardfault_handler)		/* 3 -> Hard fault */
	.word	0				/* 4 -> Reserved */
	.word	0				/* 5 -> Reserved */
	.word	0				/* 6 -> Reserved */
	.word	0				/* 7 -> Reserved */
	.word	0				/* 8 -> Reserved */
	.word	0				/* 9 -> Reserved */
	.word	0				/* 10 -> Reserved */
	.word	0				/* 11 -> Svcall */
	.word	0				/* 12 -> Reserved */
	.word	0				/* 13 -> Reserved */
	.word	Csym(knl_dispatch_entry)	/* 14 -> Pend SV */
	.word	Csym(knl_timer_handler_startup)	/* 15 -> Systick */
	/* External Interrupts */
__extintvec:
	.word	Csym(knl_inthdr_startup)	/* 0 */
	.word	Csym(knl_inthdr_startup)	/* 1 */
	.word	Csym(knl_inthdr_startup)	/* 2 */
	.word	Csym(knl_inthdr_startup)	/* 3 */
	.word	Csym(knl_inthdr_startup)	/* 4 */
	.word	Csym(knl_inthdr_startup)	/* 5 */
	.word	Csym(knl_inthdr_startup)	/* 6 */
	.word	Csym(knl_inthdr_startup)	/* 7 */
	.word	Csym(knl_inthdr_startup)	/* 8 */
	.word	Csym(knl_inthdr_startup)	/* 9 */
	.word	Csym(knl_inthdr_startup)	/* 10 */
	.word	Csym(knl_inthdr_startup)	/* 11 */
	.word	Csym(knl_inthdr_startup)	/* 12 */
	.word	Csym(knl_inthdr_startup)	/* 13 */
	.word	Csym(knl_inthdr_startup)	/* 14 */
	.word	Csym(knl_inthdr_startup)	/* 15 */
	.word	Csym(knl_inthdr_startup)	/* 16 */
	.word	Csym(knl_inthdr_startup)	/* 17 */
	.word	Csym(knl_inthdr_startup)	/* 18 */
	.word	Csym(knl_inthdr_startup)	/* 19 */
	.word	Csym(knl_inthdr_startup)	/* 20 */
	.word	Csym(knl_inthdr_startup)	/* 21 */
	.word	Csym(knl_inthdr_startup)	/* 22 */
	.word	Csym(knl_inthdr_startup)	/* 23 */
	.word	Csym(knl_inthdr_startup)	/* 24 */
	.word	Csym(knl_inthdr_startup)	/* 25 */
	.word	Csym(knl_inthdr_startup)	/* 26 */
	.word	Csym(knl_dispatch_to_schedtsk)	/* 27 */
	.word	Csym(knl_inthdr_startup)	/* 28 */
	.word	Csym(knl_inthdr_startup)	/* 29 */
	.word	Csym(knl_inthdr_startup)	/* 30 */
	.word	Csym(knl_inthdr_startup)	/* 31 */

/*
 * ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 * ;;
 * ;; Default interrupt handlers.
 * ;;
 */
	.text
	.align 2

/*
 * NMI handler
 */
	.thumb
 	.thumb_func
	.globl	Csym(nmi_handler)
Csym(nmi_handler):
	b	.

/*
 * Hard fault handler.
 */
	.thumb
 	.thumb_func
	.globl	Csym(hardfault_handler)
Csym(hardfault_handler):
	b	.

	.end
