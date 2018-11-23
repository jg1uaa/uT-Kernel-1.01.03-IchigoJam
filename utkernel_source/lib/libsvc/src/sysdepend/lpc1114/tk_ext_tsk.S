/*
 *	micro T-Kernel SVC interface library (lpc1114)
 */

#include <machine.h>
#include <tk/sysdef.h>
#include <sys/svc/tkfncd.h>
#include "utk_config.h"

	.code 16
	.syntax unified
	.thumb

#if USE_TRAP || (USE_DBGSPT && USE_HOOK_TRACE)
	.text
	.align 2
	.thumb_func
	.globl	Csym(tk_ext_tsk)
	.type	Csym(tk_ext_tsk), %function
Csym(tk_ext_tsk):
	push	{r4,lr}
	add	r4, sp, #8
	ldr	ip, =TFN_EXT_TSK
#if USE_TRAP
	svc	SVC_SYSCALL
#else
	bl	Csym(knl_call_entry)
#endif
	pop	{r4,lr}
	bx	lr
#endif

