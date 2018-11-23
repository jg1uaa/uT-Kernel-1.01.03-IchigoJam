/*
 *	micro T-Kernel SVC interface library (lpc1114)
 */

#include <machine.h>
#include <tk/sysdef.h>
#include <sys/svc/tdfncd.h>
#include "utk_config.h"

	.code 16
	.syntax unified
	.thumb

#if USE_TRAP || (USE_DBGSPT && USE_HOOK_TRACE)
	.text
	.align 2
	.thumb_func
	.globl	Csym(td_lst_cyc)
	.type	Csym(td_lst_cyc), %function
Csym(td_lst_cyc):
	push	{r4,lr}
	ldr	ip, =TDFN_LST_CYC
#if USE_TRAP
	svc	SVC_DEBUG_SUPPORT
#else
	bl	Csym(knl_call_dbgspt)
#endif
	pop	{r4,lr}
	bx	lr
#endif

