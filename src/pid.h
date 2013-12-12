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

#define OTHER   false
#define ANGULAR true

#define D_ERROR true
#define D_STATE false

// PID Variables
typedef struct PIDdata
{
    float   B, P, I, D;
    float   iTerm;
    float   windupGuard;
    float   lastDcalcValue;
    float   lastDterm;
    float   lastLastDterm;
    uint8_t dErrorCalc;
    uint8_t type;
} PIDdata_t;

extern uint8_t holdIntegrators;

///////////////////////////////////////////////////////////////////////////////

void initPID(void);

///////////////////////////////////////////////////////////////////////////////

float updatePID(float command, float state, float deltaT, uint8_t iHold, struct PIDdata *PIDparameters);

///////////////////////////////////////////////////////////////////////////////

void setPIDintegralError(uint8_t IDPid, float value);

///////////////////////////////////////////////////////////////////////////////

void zeroPIDintegralError(void);

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value);

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void);

///////////////////////////////////////////////////////////////////////////////



