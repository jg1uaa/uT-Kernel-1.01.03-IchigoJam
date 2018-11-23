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
 *	cpu_insn.h (LPC1114)
 *	LPC1114-Dependent Operation
 */

#ifndef _CPU_INSN_
#define _CPU_INSN_

#include <sys/sysinfo.h>

/* ------------------------------------------------------------------------ */
/*
 *	Control register operation
 */

Inline UINT knl_getXPSR(void)
{
	UINT	xpsr;

	Asm("mrs %0, xpsr": "=r"(xpsr));

	return xpsr;
}

Inline UINT knl_getPRIMASK(void)
{
	UINT	primask;

	Asm("mrs %0, primask": "=r"(primask));

	return primask;
}

/* ------------------------------------------------------------------------ */
/*
 *	EIT-related
 */


/*
 * Set interrupt handler
 */
Inline void knl_define_inthdr( INT vecno, FP inthdr )
{
	/* do nothing */
}

/*
 * If it is the task-independent part, TRUE
 */
Inline BOOL knl_isTaskIndependent( void )
{
	return ( knl_taskindp > 0 )? TRUE: FALSE;
}

/*
 * Move to/Restore task independent part
 */
Inline void knl_EnterTaskIndependent( void )
{
	knl_taskindp++;
}
Inline void knl_LeaveTaskIndependent( void )
{
	knl_taskindp--;
}

/* ------------------------------------------------------------------------ */

#endif /* _CPU_INSN_ */
