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

///////////////////////////////////////////////////////////////////////////////
// CLI Initialization
///////////////////////////////////////////////////////////////////////////////

void cliInit(void)
{
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
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
    uint32_t oldTxed;
    uint32_t start;
    uint32_t txed;

    if (!(usbIsConnected() && usbIsConfigured()) || !str)
    {
	    return;
	}

	len     = strlen(str);

	txed    = 0;
	oldTxed = 0;

	start   = millis();

	while (txed < len && (millis() - start < USB_TIMEOUT))
	{
	    txed += CDC_Send_DATA((uint8_t*)str + txed, len - txed);

	    if (oldTxed != txed)
	    {
	        start = millis();
	    }

	    oldTxed = txed;
	}
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
