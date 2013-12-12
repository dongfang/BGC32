/*

BGC32 from FocusFlight, a new alternative firmware
for the EvvGC controller

Original work Copyright (c) 2013 John Ihlein
                                 Alan K. Adamson

This file is part of BGC32.

Includes code and/or ideas from:

  1)BaseFlight
  2)EvvGC
  2)S.O.H. Madgwick

BGC32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BGC32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with EvvGC. If not, see <http://www.gnu.org/licenses/>.

*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

///////////////////////////////////////

#include "pid.h"

#include "bgc32.h"

#include "drv_cli.h"
#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_irq.h"
#include "drv_pwmMotors.h"
#include "drv_rc.h"
#include "drv_system.h"
#include "drv_timingFunctions.h"
#include "drv_usart.h"
#include "ringbuffer.h"

#include "hmc5883.h"
#include "mpu6050.h"

#include "cli.h"
#include "computeMotorCommands.h"
#include "config.h"
#include "fastTrig.h"
#include "firstOrderFilter.h"
#include "magCalibration.h"
#include "MargAHRS.h"
#include "mpu6050Calibration.h"
#include "pointingCommands.h"
#include "utilities.h"

///////////////////////////////////////////////////////////////////////////////
