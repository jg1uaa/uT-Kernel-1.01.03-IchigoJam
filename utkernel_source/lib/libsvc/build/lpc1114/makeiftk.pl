#! /usr/bin/env perl
#
# ----------------------------------------------------------------------
#     micro T-Kernel 2.0 Software Package
#
#     Copyright (C) 2006-2014 by Ken Sakamura.
#     This software is distributed under the T-License 2.0.
# ----------------------------------------------------------------------
#
#     Released by T-Engine Forum(http://www.t-engine.org/) at 2014/09/01.
#
# ----------------------------------------------------------------------
#

#
#	makeiftk.pl
#
#	generate interface library
#

sub makelib
{
	print LIB <<EndOfIfLibBody;
#include <machine.h>
#include <tk/sysdef.h>
#include <sys/svc/$fn_h>
#include "utk_config.h"

	.code 16
	.syntax unified
	.thumb

#if USE_TRAP || (USE_DBGSPT && USE_HOOK_TRACE)
	.text
	.align 2
	.thumb_func
	.globl	Csym(${func})
	.type	Csym(${func}), %function
Csym(${func}):
	push	{r4,lr}
	add	r4, sp, #8
	ldr	ip, =TFN_${Func}
#if USE_TRAP
	svc	SVC_SYSCALL
#else
	bl	Csym(knl_call_entry)
#endif
	pop	{r4,lr}
	bx	lr
#endif

EndOfIfLibBody
}

1;
