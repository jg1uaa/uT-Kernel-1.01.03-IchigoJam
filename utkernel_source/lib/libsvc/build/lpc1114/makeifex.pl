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
#	makeifex.pl
#
#	generate extended SVC interface library
#

sub makelibex
{
	print LIB <<EndOfExtIfLibBody;
#include <machine.h>
#include <tk/sysdef.h>
#include <sys/svc/$fn_h>
#include "utk_config.h"

	.code 16
	.syntax unified
	.thumb

	IF	(USE_TRAP != 0) :LOR: ( (USE_DBGSPT != 0) :LAND: (USE_HOOK_TRACE != 0) )
	.text
	.align 2
	.thumb_func
	.globl Csym(${func})
	.type	Csym(${func}), %function
Csym(${func}):
	stmfd	sp!, {r0-r3}
	mov	r0, sp
	stmfd	sp!, {lr}
	ldr	ip, =${prefix}_${Func}_FN
	svc	SVC_EXTENDED_SVC
	ldmfd	sp!, {lr}
	add	sp, sp, #4*4
	bx	lr

EndOfExtIfLibBody
}

1;
