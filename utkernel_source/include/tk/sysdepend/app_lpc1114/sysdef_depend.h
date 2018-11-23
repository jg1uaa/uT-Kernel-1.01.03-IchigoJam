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
 *	@(#)sysdef_depend.h (LPC1114)
 *
 *	Definition about LPC1114
 *
 *	Included also from assembler program.
 */

#ifndef __TK_SYSDEF_DEPEND_H__
#define __TK_SYSDEF_DEPEND_H__

/*
 * Program status register (PSR)
 */
#define PSR_N		0x80000000	/* Condition flag Negative */
#define PSR_Z		0x40000000	/* Zero */
#define PSR_C		0x20000000	/* Carry */
#define PSR_V		0x10000000	/* Overflow */
#define PSR_Q		0x08000000	/* Saturation */

#define PSR_INT_MSK	0x000000FF	/* Interrupt status mask */

/* Exception model. */
#define EXP_M(n)	(n)		/* Exception model */
#define EXP_USR		EXP_M(0)	/* User mode, No Exception */
#define EXP_RST		EXP_M(1)	/* Reset */
#define EXP_NMI		EXP_M(2)	/* Non Maskable Interrupt */
#define EXP_HDF		EXP_M(3)	/* Hardware fault */
#define EXP_MEM		EXP_M(4)	/* Memory fault */
#define EXP_BUS		EXP_M(5)	/* Bus fault */
#define EXP_USF		EXP_M(6)	/* Usage fault */
#define EXP_SVC		EXP_M(11)	/* SVC call */
#define EXP_DBG		EXP_M(12)	/* Debug monitor */
#define EXP_PSV		EXP_M(14)	/* Software asynchronous System call */
#define EXP_STK		EXP_M(15)	/* System tick */
#define EXP_EXT(n)	(EXP_M(16) + n)	/* External interrupt */


/* ------------------------------------------------------------------------ */

/*
 * Parameter of SVC instruction
 */
#define SVC_SYSCALL		0x00	/* micro T-Kernel system call/extension SVC */
#define SVC_EXTENDED_SVC	0x10	/* Extended SVC */
#define SVC_DEBUG_SUPPORT	0xFF	/* Debugger support function */

/* ------------------------------------------------------------------------ */

/*
 * System control block
 */
#define SCB_ICSR	0xE000ED04
#define SCB_AIRCR	0xE000ED0C

#define SCB_SCR		0xE000ED10
#define SCB_CCR		0xE000ED14
#define SCB_SHPR2	0xE000ED1C
#define SCB_SHPR3	0xE000ED20

#define ICSR_PENDSVSET	0x10000000	/* Trigger PendSV exception. */
#define ICSR_PENDSVCLR	0x08000000	/* Remove the pending state from the PendSV exception. */
#define ICSR_PENDSTCLR	0x02000000	/* Remove the pending state from the SysTick exception. */


/*
 * System Timer
 */
#define SYST_CSR	0xE000E010	/* SysTick Control and Status */
#define SYST_RVR	0xE000E014	/* SysTick Reload value */
#define SYST_CVR	0xE000E018	/* SysTick Current value */

/* ------------------------------------------------------------------------ */

/*
 * NVIC (Nested Vectored Interrupt Controller)
 */
#define NVIC_ISER_BASE	0xE000E100
#define NVIC_ISER(x)	(NVIC_ISER_BASE + (((x) / 32) << 2))

#define NVIC_ICER_BASE	0xE000E180
#define NVIC_ICER(x)	(NVIC_ICER_BASE + (((x) / 32) << 2))

#define NVIC_ISPR_BASE	0xE000E200
#define NVIC_ISPR(x)	(NVIC_ISPR_BASE + (((x) / 32) << 2))

#define NVIC_ICPR_BASE	0xE000E280
#define NVIC_ICPR(x)	(NVIC_ICPR_BASE + (((x) / 32) << 2))

#define NVIC_IPR_BASE	0xE000E400
#define NVIC_IPR(x)	(NVIC_IPR_BASE  + (((x) /  4) << 2))

/* ------------------------------------------------------------------------ */
/*
 * force dispatch interrupt for micro T-Kernel
 */
#define INTNO_FORCE_DISPATCH	27
#define NVIC_IPR_FORCE_DISPATCH	NVIC_IPR(INTNO_FORCE_DISPATCH)

/* ------------------------------------------------------------------------ */

#endif /* __TK_SYSDEF_DEPEND_H__ */
