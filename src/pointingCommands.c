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
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float    pointingCmd[3] = { 0.0f, 0.0f, 0.0f };

float    rxCommand[3]   = { 0.0f, 0.0f, 0.0f };

uint8_t  commandInDetent[3]         = { true, true, true };
uint8_t  previousCommandInDetent[3] = { true, true, true };

///////////////////////////////////////////////////////////////////////////////
// Process Pointing Commands
///////////////////////////////////////////////////////////////////////////////

void processPointingCommands(void)
{
    uint8_t channel;

    if (rcActive == true)
    {
		// Read receiver commands
        for (channel = 0; channel < 3; channel++)
           rxCommand[channel] = (float)rxRead(channel);

        rxCommand[ROLL]  -= eepromConfig.midCommand;                  // Roll Range    -1000:1000
        rxCommand[PITCH] -= eepromConfig.midCommand;                  // Pitch Range   -1000:1000
        rxCommand[YAW]   -= eepromConfig.midCommand;                  // Yaw Range     -1000:1000
    }

    // Set past command in detent values
    for (channel = 0; channel < 3; channel++)
    	previousCommandInDetent[channel] = commandInDetent[channel];

    // Apply deadbands and set detent discretes
    for (channel = 0; channel < 3; channel++)
    {
    	if ((rxCommand[channel] <= DEADBAND) && (rxCommand[channel] >= -DEADBAND))
        {
            rxCommand[channel] = 0;
  	        commandInDetent[channel] = true;
  	    }
        else
  	    {
  	        commandInDetent[channel] = false;
  	        if (rxCommand[channel] > 0)
  	        {
  		        rxCommand[channel] = (rxCommand[channel] - DEADBAND) * DEADBAND_SLOPE;
  	        }
  	        else
  	        {
  	            rxCommand[channel] = (rxCommand[channel] + DEADBAND) * DEADBAND_SLOPE;
  	        }
        }
    }

    ///////////////////////////////////

    rxCommand[ROLL]  *=  0.001f;  // Roll Range  -1:1
    rxCommand[PITCH] *= -0.001f;  // Pitch Range -1:1
    rxCommand[YAW]   *=  0.001f;  // Yaw Range   -1:1

    ///////////////////////////////////

    if (eepromConfig.rollRateCmdInput == true)
    {
        if ((rxCommand[ROLL] >= 0.0f) && (pointingCmd[ROLL] <=  eepromConfig.gimbalRollRightLimit))
            pointingCmd[ROLL] += rxCommand[ROLL] * eepromConfig.gimbalRollRate * 0.02f;  // Constant DT of 0.02 good enough here

        if ((rxCommand[ROLL] < 0.0f) && (pointingCmd[ROLL] >= -eepromConfig.gimbalRollLeftLimit))
            pointingCmd[ROLL] += rxCommand[ROLL] * eepromConfig.gimbalRollRate * 0.02f;  // Constant DT of 0.02 good enough here

        pointingCmd[ROLL] = firstOrderFilter(pointingCmd[ROLL], &firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS]);
    }
	else
	{
        if (rxCommand[ROLL] >= 0.0f)
            pointingCmd[ROLL] = rxCommand[ROLL] * eepromConfig.gimbalRollRightLimit;
        else
            pointingCmd[ROLL] = rxCommand[ROLL] * eepromConfig.gimbalRollLeftLimit;

        pointingCmd[ROLL] = firstOrderFilter(pointingCmd[ROLL], &firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS]);
	}

	///////////////////////////////////

	if (eepromConfig.pitchRateCmdInput == true)
	{
        if ((rxCommand[PITCH] >= 0.0f) && (pointingCmd[PITCH] <=  eepromConfig.gimbalPitchUpLimit))
            pointingCmd[PITCH] += rxCommand[PITCH] * eepromConfig.gimbalPitchRate * 0.02f;  // Constant DT of 0.02 good enough here

        if ((rxCommand[PITCH] < 0.0f) && (pointingCmd[PITCH] >= -eepromConfig.gimbalPitchDownLimit))
            pointingCmd[PITCH] += rxCommand[PITCH] * eepromConfig.gimbalPitchRate * 0.02f;  // Constant DT of 0.02 good enough here

        pointingCmd[PITCH] = firstOrderFilter(pointingCmd[PITCH], &firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS]);
}
	else
	{
        if (rxCommand[PITCH] >= 0.0f)
            pointingCmd[PITCH] = rxCommand[PITCH] * eepromConfig.gimbalPitchUpLimit;

        if (rxCommand[PITCH] < 0.0f)
            pointingCmd[PITCH] = rxCommand[PITCH] * eepromConfig.gimbalPitchDownLimit;

        pointingCmd[PITCH] = firstOrderFilter(pointingCmd[PITCH], &firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS]);
	}

	///////////////////////////////////

	if (eepromConfig.yawRateCmdInput == true)
	{
        if ((rxCommand[YAW] >= 0.0f) && (pointingCmd[YAW] <=  eepromConfig.gimbalYawRightLimit))
            pointingCmd[YAW] += rxCommand[YAW] * eepromConfig.gimbalYawRate * 0.02f;  // Constant DT of 0.02 good enough here

        if ((rxCommand[YAW] < 0.0f) && (pointingCmd[YAW] >= -eepromConfig.gimbalYawLeftLimit))
            pointingCmd[YAW] += rxCommand[YAW] * eepromConfig.gimbalYawRate * 0.02f;  // Constant DT of 0.02 good enough here

        pointingCmd[YAW] = firstOrderFilter(pointingCmd[YAW], &firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS]);
    }
	else
	{
        if (rxCommand[YAW] >= 0.0f)
            pointingCmd[YAW] = rxCommand[YAW] * eepromConfig.gimbalYawRightLimit;

        if (rxCommand[YAW] < 0.0f)
            pointingCmd[YAW] = rxCommand[YAW] * eepromConfig.gimbalYawLeftLimit;

        pointingCmd[YAW] = firstOrderFilter(pointingCmd[YAW], &firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS]);
    }

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////




