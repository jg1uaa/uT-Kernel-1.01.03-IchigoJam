/*
 *----------------------------------------------------------------------
 *    micro T-Kernel
 *
 *    Copyright (C) 2006-2014 by Ken Sakamura.
 *    This software is distributed under the T-License 2.0.
 *----------------------------------------------------------------------
 *
 *    Version:   1.01.03
 *    Released by T-Engine Forum(http://www.t-engine.org/) at 2014/09/01.
 *
 *----------------------------------------------------------------------
 */

/*
 *	inittask_def.h
 *	Initial task definition
 */

#ifndef _INITTASK_DEF_
#define _INITTASK_DEF_

#include <basic.h>
#include <tk/tkernel.h>
#include "utk_config.h"

/*
 * Initial task parameter
 */
#define INITTASK_EXINF		(0x0)
#define INITTASK_TSKATR		(TA_HLNG | TA_RNG0)
#define INITTASK_ITSKPRI	(INIT_TASK_PRI)
#define INITTASK_STKSZ		(1*1024)
#define INITTASK_DSNAME		"inittsk"
#define INITTASK_STACK		(NULL)

#endif /* _INITTASK_DEF_ */
