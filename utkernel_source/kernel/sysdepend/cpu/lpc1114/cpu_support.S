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
 *	cpu_support.S (LPC1114)
 *	Device-Dependent CPU Operation
 */

#define	_in_asm_source_

#include "utk_config.h"

#include <machine.h>
#include <tk/errno.h>
#include <tk/sysdef.h>
#include <tk/asm.h>
#include <sys/sysinfo.h>

#if USE_TRAP | USE_DBGSPT | USE_HOOK_TRACE
#error ouch!
#endif

#include "tkdev_conf.h"
#include "offset.h"

	.code 16
	.syntax unified
	.thumb

/* ------------------------------------------------------------------------ */
/*
 * Dispatcher
 *	dispatch_entry:
 *		Normal dispatch processing.
 *	_ret_int_dispatch:
 *		Called when dispatch is required by 'tk_ret_int().'
 *
 *	Contexts to save
 *	Save registers except for ssp to a stack. Save 'ssp' to TCB.
 *
 *	   High Address +---------------+
 *			| xpsr		|
 *			| Return Address| Return address (pc)
 *			| lr		|
 *			| R12 (ip)	|
 *			| R0-R3		|
 *			+---------------+ Save by Exception entry process.
 *			+---------------+
 *			| lr		|
 *			| taskmode	|
 *		ssp ->	| R4 - R11	|
 *	    Low Address +---------------+
 */
	.text
	.align 2
	.thumb
	.thumb_func
	.globl	Csym(knl_dispatch_to_schedtsk)
Csym(knl_dispatch_to_schedtsk):
	/* Interrupt is disabled(CPSR.I=1 F=1), during SVC mode */
	ldr	r0, =TMP_STACK_TOP		/* Set temporal stack */
	mov	sp, r0

	ldr	r3, =Csym(knl_dispatch_disabled)
	movs	r0, #1
	str	r0, [r3]			/* Dispatch disable */

	ldr	r4, =Csym(knl_ctxtsk)		/* R4 = &ctxtsk */
	movs	r0, #0
	str	r0, [r4]			/* ctxtsk = NULL */

	/* Clear the pending state from the PendSV exception */
	ldr	r0, =SCB_ICSR
	ldr	r3, =ICSR_PENDSVCLR
	str	r3, [r0]

	b	l_dispatch0

	.thumb
	.thumb_func
	.globl	Csym(knl_dispatch_entry)
Csym(knl_dispatch_entry):
_ret_int_dispatch:
	ldr	r0, =Csym(knl_dispatch_disabled)
	movs	r1, #1
	str	r1, [r0]			/* Dispatch disable */

	ldr	r0, =Csym(knl_taskmode)
	ldr	r0, [r0]
	push	{r0, lr}			/* Context save (lr) */

	mov	r0, r8
	mov	r1, r9
	mov	r2, r10
	mov	r3, r11
	push	{r0-r3}
	push	{r4-r7}

	/* Context save */
	ldr	r4, =Csym(knl_ctxtsk)		/* R4 = &ctxtsk */
	movs	r0, #0
	ldr	r7, [r4]
	cmp	r7, #0
	beq	0f
	mov	r1, sp
	str	r1, [r7, #TCB_tskctxb + CTXB_ssp]	/* Save 'ssp' to TCB */
0:
	str	r0, [r4]			/* ctxtsk = NULL */

l_dispatch0:
	ldr	r5, =Csym(knl_schedtsk)		/* R5 = &schedtsk */
	ldr	r6, =Csym(knl_lowpow_discnt)	/* R6 = &lowpow_discnt */

l_dispatch1:
	cpsid	i

	ldr	r7, [r5]			/* R7 = schedtsk */
	cmp	r7, #0				/* Is there 'schedtsk'? */
	bne	l_dispatch2

	/* Because there is no task that should be executed, move to the power-saving mode */
	ldr	r3, [r6]			/* Is 'low_pow' disabled? */
	cmp	r3, #0
	cpsie	i
	bne	1f
	wfi					/* do low_pow() */
1:
	b	l_dispatch1

l_dispatch2:					/* Switch to 'schedtsk' */
	str	r7, [r4]			/* ctxtsk = schedtsk */
	ldr	r1, [r7, #TCB_tskctxb + CTXB_ssp]	/* Restore 'ssp' from TCB */
	mov	sp, r1

	ldr	r0, =Csym(knl_dispatch_disabled)
	movs	r1, #0
	str	r1, [r0]			/* Dispatch enable */

	pop	{r4-r7}
	pop	{r0-r3}
	mov	r8, r0
	mov	r9, r1
	mov	r10, r2
	mov	r11, r3

	pop	{r0, r3}
	ldr	r1, =Csym(knl_taskmode)
	str	r0, [r1]

	mov	lr, r3				/* R14_svc restore */

	cpsie	i

	EXC_RETURN

#if USE_HLL_INTHDR
/* ------------------------------------------------------------------------ */
/*
 * High level programming language routine for interrupt handler
 *	Called by interrupt entry routine.
 */
	.text
	.align 2
	.thumb
	.thumb_func
	.globl	Csym(knl_inthdr_startup)
Csym(knl_inthdr_startup):
	push	{r4,lr}			/* push EXC_RETURN value to stack */
					/* r4 is dummy (AAPCS) */
	ldr	r3, =Csym(knl_taskindp)	/* Task independent part */
	ldr	r1, [r3]
	adds	r1, r1, #1
	str	r1, [r3]

	mrs	r0, ipsr
	subs	r0, r0, #16		/* interrupt number (dintno) */

	ldr	r3, =Csym(knl_hll_inthdr)
	lsls	r1, r0, #2
	ldr	r3, [r3, r1]
	blx	r3			/* call hll_inthdr[n](dintno) */

	ldr	r3, =Csym(knl_taskindp)
	ldr	r2, [r3]
	subs	r2, r2, #1
	str	r2, [r3]

	pop	{r4,pc}			/* pop EXC_RETURN value from stack */
					/* r4 is dummy (AAPCS) */

#endif /* USE_HLL_INTHDR */

/*
 * tk_ret_int_impl()
 */
	.text
	.align 2
	.thumb
	.thumb_func
	.globl	Csym(tk_ret_int_impl)
Csym(tk_ret_int_impl):
	EXC_RETURN

/* ------------------------------------------------------------------------ */
/*
 * High level programming language routine for timer handler
 */
	.text
	.align 2
	.thumb
	.thumb_func
	.globl	Csym(knl_timer_handler_startup)
Csym(knl_timer_handler_startup):

	push	{r3-r5, lr}		/* Register save */

	ldr	r4, =Csym(knl_taskindp)	/* Enter task independent part */
	ldr	r5, [r4]
	adds	r0, r5, #1
	str	r0, [r4]

	bl	Csym(knl_timer_handler)	/* call timer_handler() */

	str	r5, [r4]		/* Leave task independent part */

	pop	{r3-r5, pc}		/* Register restore */

/* ------------------------------------------------------------------------ */

/*
 * Throw away the current task context.
 * and forcibly dispatch to the task that should be performed next.
 *	Use at system startup and 'tk_ext_tsk, tk_exd_tsk.'
 */

	.thumb
	.thumb_func
	.globl	Csym(knl_force_dispatch)
Csym(knl_force_dispatch):
	ldr	r0, =(1 << (INTNO_FORCE_DISPATCH % 32))
	ldr	r1, =NVIC_ISPR(INTNO_FORCE_DISPATCH)	/* Generate S/W Interrupt */
	str	r0, [r1]
	dsb

	movs	r0, #0
	bl	Csym(enaint)
	bx	lr

	.thumb
	.thumb_func
	.globl	Csym(knl_dispatch)
Csym(knl_dispatch):
	ldr	r0, =SCB_ICSR
	ldr	r1, =ICSR_PENDSVSET	/* Trigger pendsv exception */
	str	r1, [r0]
	bx	lr

/*
 * Clear timer interrupt
 *	Clear the timer interrupt request. Depending on the type of
 *	hardware, there are two timings for clearing: at the beginning
 *	and the end of the interrupt handler.
 *	'clear_hw_timer_interrupt()' is called at the beginning of the
 *	timer interrupt handler.
 *	'end_of_hw_timer_interrupt()' is called at the end of the timer
 *	interrupt handler.
 *	Use either or both according to hardware.
 */
	.thumb
	.thumb_func
	.globl	Csym(knl_clear_hw_timer_interrupt)
Csym(knl_clear_hw_timer_interrupt):
	ldr	r0, =SCB_ICSR
	ldr	r1, =ICSR_PENDSTCLR
	str	r1, [r0]
	bx	lr

	.end
