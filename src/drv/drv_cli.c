/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

#define USB_TIMEOUT  50

tRingBuffer RingBufferUSBTX;

int USBCallBackCalled = 0;

unsigned long lastCallbackTime;

///////////////////////////////////////////////////////////////////////////////

int usbOverrun(void)
{
    return (RingBufferUSBTX.Overrun);
}

///////////////////////////////////////////////////////////////////////////////

void USBPushTXData(void)
{
    tRingBuffer *rb = &RingBufferUSBTX;
    uint8_t *p = rb->Buffer + rb->Read;
    int len = rb->Write - rb->Read;

    if (len != 0)
    {
        if (len < 0)
        {
            len = RingBufferSize(rb) - rb->Read;
        }

        len = CDC_Send_DATA(p, len);
        rb->Read = (rb->Read + len) % RingBufferSize(rb);
    }
}

///////////////////////////////////////////////////////////////////////////////

void USBPushTX(void)
{
    if (packetSent)
    {
        return; // transfer will be handled by next callback
    }

    // something hangs, retrigger send
    lastCallbackTime = millis();
    USBPushTXData();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Initialization
///////////////////////////////////////////////////////////////////////////////

void cliInit(void)
{
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();

	RingBufferInit(&RingBufferUSBTX, &USBPushTX);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Available
///////////////////////////////////////////////////////////////////////////////

uint32_t cliAvailable(void)
{
    return receiveLength;
}

///////////////////////////////////////////////////////////////////////////////
// CLI Read
///////////////////////////////////////////////////////////////////////////////

uint8_t cliRead(void)
{
    uint8_t buf[1];

    uint32_t rxed = 0;

    while (rxed < 1)
    {
        rxed += CDC_Receive_DATA((uint8_t*)buf + rxed, 1 - rxed);
    }

    return buf[0];
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print
///////////////////////////////////////////////////////////////////////////////

void cliPrint(char *str)
{
    uint32_t len;

    len = strlen(str);

	if (usbIsConnected())
	    RingBufferPutBlock(&RingBufferUSBTX, (uint8_t *)str, len, 0);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print Formatted - Print formatted string to USB VCP
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void cliPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	cliPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
