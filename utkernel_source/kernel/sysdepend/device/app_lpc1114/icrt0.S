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
 *  @(#) icrt0.S
 *
 *    Start up module
 *
 */

#include "hwconfig.h"
#include "utk_config.h"

#include <machine.h>
#include <tk/asm.h>

	.code 16
	.syntax unified
	.thumb

	.text
	.align 2
	.thumb_func
	.globl Csym(Reset_Handler)
Csym(Reset_Handler):

data_init:	/* .data */
	ldr	r1, =__data_org		/* src address */
	ldr	r2, =__data_start	/* dst address */
	ldr	r3, =__data_end
	subs	r3, r3, r2		/* r3 := data_size */
	beq	data_done		/* if __data_start == __data_end */

data_loop:
	ldmia	r1!, {r0}
	stmia	r2!, {r0}
	subs	r3, r3, #4
	bne	data_loop		/* if data_size != 0 */

data_done:

bss_init:	/* .bss */
#if USE_NOINIT
	ldr     r2, =__noinit_end	/* dst address */
#else
	ldr     r2, =__bss_start	/* dst address */
#endif
	ldr     r3, =__bss_end
	subs    r3, r3, r2		/* r3 := data_size */
	beq     bss_done		/* if __bss_start == __bss_end */
	movs	r0, #0			/* r0 := 0 */

bss_loop:
	stmia	r2!, {r0}
	subs    r3, r3, #4
	bgt     bss_loop		/* if data_size > 0 */

bss_done:

#if USE_IMALLOC
	ldr     r5, =SYSTEMAREA_TOP
	cmp	r2, r5			/* _end or RAM_TOP */
	bls	0f
	mov	r5, r2			/* Either of High addresses */
0:	
	ldr	r0, =Csym(knl_lowmem_top)
	str	r5, [r0]		/* knl_lowmem_top = _end or RAM_TOP */
	ldr     r5, =SYSTEMAREA_END - (EXC_STACK_SIZE + TMP_STACK_SIZE)
	ldr	r0, =Csym(knl_lowmem_limit)
	str	r5, [r0]		/* knl_lowmem_limit = RAM_END */
#endif

	/* Configure exception priorities */
	ldr	r5, =SCB_SHPR2
	movs	r6, #0			/* SVC pri = 0 */
	str	r6, [r5]

	ldr	r5, =SCB_SHPR3
	ldr	r6, =0x40C00000		/* Pendsv pri = 3, systick pri = 1 */
	str	r6, [r5]

	/* Enable software interrupt */
	movs	r0, #INTNO_FORCE_DISPATCH
	movs	r1, #0xc0		/* force dispatch pri = 3 */
	bl	Csym(EnableInt)

	/* Allow user to access SCB_STIR */
	ldr	r5, =SCB_CCR
	ldr	r6, [r5]
	movs	r7, #0x02
	orrs	r6, r6, r7
	str	r6, [r5]

kernel_start:
	bl	Csym(init_clock_control)
	bl	Csym(main)

l_end:
	b	.

