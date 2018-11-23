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
#	Makefile for gmake
#	sysmain (system-dependent description)
#

# T-Engine type
MACHINE = lpc1114
TETYPE	= app

# default rules
include ../../../../etc/makerules

# source files (system-dependent)
SRC	= power.c clock_mode.c

# Manager and Driver objects (links to the kernel)
K_OBJ	=
TARGET_CRT0_RAM	=
TARGET_CRT0_ROM	=
TARGET_CRT0_COMMON	= icrt0.o vector.o

# include main makefile (common description)
include ../../src/Makefile.common

# create intel hex object
ALL += $(TARGET_ROM:%.rom=%.hex)

$(TARGET_ROM:%.rom=%.hex): $(TARGET_ROM)
	$(OBJCOPY) $(OUTPUT_IHEX) $< $@
