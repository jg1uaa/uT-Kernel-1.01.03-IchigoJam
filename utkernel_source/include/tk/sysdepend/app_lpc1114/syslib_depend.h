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
 *	@(#)syslib_depend.h (LPC1114)
 *
 *	micro T-Kernel LPC1114 Library
 */

#ifndef __TK_SYSLIB_DEPEND_H__
#define __TK_SYSLIB_DEPEND_H__

#include <tk/errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CPU interrupt control
 *	As armv6-m architecture does not support disable interrupt in
 *	xpsr register, write the PRIMASK to implement disint and enaint.
 */
IMPORT UINT disint( void );
IMPORT UINT enaint( UINT intsts );
#define DI(intsts)	( (intsts) = disint() )
#define EI(intsts)	( enaint(intsts) )
#define isDI(intsts)	( (intsts) != 0 )

/*
 * Interrupt vector
 *	The interrupt vector is the index number of the vector table.
 */
typedef UINT	INTVEC;

/* Convert to interrupt definition number */
#define DINTNO(intvec)	(intvec)

/*
 * Interrupt enable
 *	Enable the interrupt specified by 'intvec.'
 */
IMPORT void EnableInt( INTVEC intvec, INT intpri );

/*
 * Interrupt disable
 *	Disable the interrupt specified by 'intvec.'
 */
IMPORT void DisableInt( INTVEC intvec );

/*
 * Clear interrupt
 *
 *	Clear the state that the specified interrupt has been
 *	asserted. 
 *
 *	We just clear the pending state from the specified interrupt.
 *	Because the processor automatically removes the pending state
 *	from the interrupt when it enters the interrupt service
 *	routine, and removes the active state from the interrupt when
 *	it returns from the interrupt service routine.
 */
#define ClearInt(intvec) ClearPendingInt(intvec)

/*
 * Issue EOI to Interrupt Controller.
 */
#define EndOfInt(intvec)

/*
 * Check active state
 *	Current active state for the associated interrupt
 */
IMPORT BOOL CheckInt( INTVEC intvec );

/* ------------------------------------------------------------------------ */
/*
 * Interrupt Controller functions for Cortex-M0.
 */

/*
 * Set-Pending
 *  Pends the associated interrupt under software control.
 */
IMPORT void SetPendingInt( INTVEC intvec );

/*
 * Clear-Pending
 *	Un-pends the associated interrupt under software control.
 */
IMPORT void ClearPendingInt( INTVEC intvec );

/* ------------------------------------------------------------------------ */

/*
 * I/O port access
 *	Only memory mapped I/O for Arm
 */
Inline void out_w( UW port, UW data )
{
	*(_UW*)port = data;
}
Inline void out_h( UW port, UH data )
{
	*(_UH*)port = data;
}
Inline void out_b( UW port, UB data )
{
	*(_UB*)port = data;
}

Inline UW in_w( UW port )
{
	return *(_UW*)port;
}
Inline UH in_h( UW port )
{
	return *(_UH*)port;
}
Inline UB in_b( UW port )
{
	return *(_UB*)port;
}

#ifdef __cplusplus
}
#endif
#endif /* __TK_SYSLIB_DEPEND_H__ */
