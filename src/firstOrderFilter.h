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

#define NUMBER_OF_FIRST_ORDER_FILTERS 9

#define ACCEL_X_500HZ_LOWPASS  0
#define ACCEL_Y_500HZ_LOWPASS  1
#define ACCEL_Z_500HZ_LOWPASS  2

#define ROLL_RATE_POINTING_50HZ_LOWPASS  3
#define PITCH_RATE_POINTING_50HZ_LOWPASS 4
#define YAW_RATE_POINTING_50HZ_LOWPASS   5

#define ROLL_ATT_POINTING_50HZ_LOWPASS  6
#define PITCH_ATT_POINTING_50HZ_LOWPASS 7
#define YAW_ATT_POINTING_50HZ_LOWPASS   8

///////////////////////////////////////////////////////////////////////////////

typedef struct firstOrderFilterData
{
    float   gx1;
    float   gx2;
    float   gx3;
    float   previousInput;
    float   previousOutput;
} firstOrderFilterData_t;

firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

///////////////////////////////////////////////////////////////////////////////

void initFirstOrderFilter(void);

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters);

///////////////////////////////////////////////////////////////////////////////



