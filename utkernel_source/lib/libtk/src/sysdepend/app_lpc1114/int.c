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
 *	@(#)int.c (libtk/Cortex-M0)
 *
 *	Interrupt controller
 */

#include <basic.h>
#include <tk/syslib.h>
#include <tk/sysdef.h>

/*
 * Enable interrupt 
 *	Enables the interrupt specified in intvec.
 */
EXPORT void EnableInt( INTVEC intvec, INT intpri )
{
	UINT	imask, shift;
	UW	pri;

	DI(imask);

	/* Set interrupt priority level. */
	shift = (intvec % 4) * 8;
	pri = *(_UW*)(NVIC_IPR(intvec));
	pri &= ~(0xff << shift);
	pri |= (intpri & 0xff) << shift;
	*(_UW*)(NVIC_IPR(intvec)) = pri;

	/* Enables the specified interrupt. */
	*(_UW*)(NVIC_ISER(intvec)) = (0x01U << (intvec % 32));

	EI(imask);
}

/*
 * Disable interrupt 
 *	Disables the interrupt specified in intvec.
 */
EXPORT void DisableInt( INTVEC intvec )
{
	*(_UW*)(NVIC_ICER(intvec)) = (0x01U << (intvec % 32));
}

/*
 * Set-Pending
 *  Pends the associated interrupt under software control.
 */
EXPORT void SetPendingInt( INTVEC intvec )
{
	*(_UW*)(NVIC_ISPR(intvec)) = (0x01U << (intvec % 32));
}

/*
 * Clear-Pending
 *	Un-pends the associated interrupt under software control.
 */
EXPORT void ClearPendingInt( INTVEC intvec )
{
	*(_UW*)(NVIC_ICPR(intvec)) = (0x01U << (intvec % 32));
}

/*
 * Check active state
 *	Current active state for the associated interrupt
 */
EXPORT BOOL CheckInt( INTVEC intvec )
{
	return (*(_UW*)(NVIC_ICPR(intvec)) & (0x01U << (intvec % 32)));
}
