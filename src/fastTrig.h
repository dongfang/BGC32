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
/*
 *  fasttrig.h
 *
 *  Created on: Aug 10, 2013
 *      Author: ala42
 */
///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define SINARRAYSIZE 1024
#define SINARRAYSCALE 32767

extern short int sinDataI16[];

///////////////////////////////////////////////////////////////////////////////

void initSinArray(void);

///////////////////////////////////////////////////////////////////////////////

float fastSin(float x);

///////////////////////////////////////////////////////////////////////////////
