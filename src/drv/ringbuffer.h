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

typedef struct
{
    volatile int Read, Write, Overrun;
    unsigned char Buffer[1024];
    void (*CallBack)(void);

} tRingBuffer;

///////////////////////////////////////////////////////////////////////////////

void RingBufferInit(tRingBuffer *rb, void (*callback)(void));

///////////////////////////////////////////////////////////////////////////////

int RingBufferSize(tRingBuffer *rb);

///////////////////////////////////////////////////////////////////////////////

int RingBufferFillLevel(tRingBuffer *rb);

///////////////////////////////////////////////////////////////////////////////

void RingBufferPut(tRingBuffer *rb, unsigned char c, int block);

///////////////////////////////////////////////////////////////////////////////

void RingBufferPutBlock(tRingBuffer *rb, unsigned char *data, int dataLen, int block);

///////////////////////////////////////////////////////////////////////////////

int RingBufferGet(tRingBuffer *rb);

///////////////////////////////////////////////////////////////////////////////

int RingBufferPeek(tRingBuffer *rb);

///////////////////////////////////////////////////////////////////////////////
