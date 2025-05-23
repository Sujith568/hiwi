#******************************************************************************
#
# am_bsp.mk - Rules for building Ambiq support libraries.
#
# Copyright (c) 2023, Ambiq Micro, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# Third party software included in this distribution is subject to the
# additional license terms as defined in the /docs/licenses directory.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# This is part of revision release_sdk_3_1_1-10cda4b5e0 of the AmbiqSuite Development Package.
#
#******************************************************************************

# Make "all" the default target.
all: bsp_pins.src am_bsp_pins.c am_bsp_pins.h $(SWROOT)/mcu/$(FAMILY)/hal/am_hal_pin.h

am_bsp_pins.c: bsp_pins.src
	python3 $(SWROOT)/tools/bsp_generator/pinconfig.py bsp_pins.src C >am_bsp_pins.c

am_bsp_pins.h: bsp_pins.src
	python3 $(SWROOT)/tools/bsp_generator/pinconfig.py bsp_pins.src H >am_bsp_pins.h

$(SWROOT)/mcu/apollo3/hal/am_hal_pin.h:
	$(MAKE) -C $(SWROOT)/mcu/apollo3/hal am_hal_pin.h

$(SWROOT)/mcu/apollo3p/hal/am_hal_pin.h:
	$(MAKE) -C $(SWROOT)/mcu/apollo3p/hal am_hal_pin.h

$(SWROOT)/mcu/apollo4/hal/am_hal_pin.h:
	$(MAKE) -C $(SWROOT)/mcu/apollo4/hal am_hal_pin.h

# All makefiles use this to find the top level directory.
# Set SWROOT to help us find other Makefiles
SWROOT?=..
include $(SWROOT)/makedefs/subdirs.mk
