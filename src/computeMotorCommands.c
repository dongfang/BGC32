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

float mechanical2electricalDegrees[3] = { 1.0f, 1.0f, 1.0f };
float electrical2mechanicalDegrees[3] = { 1.0f, 1.0f, 1.0f };

float outputRate[3];

float pidCmd[3];

float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};

float yawCmd;

#define YAP_DEADBAND     2.00f  // in radians with respect to one motor pole, actual angle is (DEADBAND / numberPoles) * R2D
#define MOTORPOS2SETPNT  0.35f  // scaling factor for how fast it should move
#define AUTOPANSMOOTH   40.00f

float centerPoint = 0.0f;
float stepSmooth  = 0.0f;
float step        = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Yaw AutoPan
///////////////////////////////////////////////////////////////////////////////

//  Inputs:
//    motorPos is electrical degrees
//    setpoint is electrical degrees
//
//  Outputs:
//    autoPan is electrical degrees

float autoPan(float motorPos, float setpoint)
{
    if (motorPos < centerPoint - YAP_DEADBAND)
    {
        centerPoint = YAP_DEADBAND;
        step = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else if (motorPos > centerPoint + YAP_DEADBAND)
    {
        centerPoint = -YAP_DEADBAND;
        step = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else
    {
        step = 0.0f;
        centerPoint = 0.0f;
    }

    stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;

    return (setpoint -= stepSmooth);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Motor Commands
///////////////////////////////////////////////////////////////////////////////

void computeMotorCommands(float dt)
{
	holdIntegrators = false;

	///////////////////////////////////

	if (eepromConfig.rollEnabled == true)
	{
	    pidCmd[ROLL] = updatePID(pointingCmd[ROLL] * mechanical2electricalDegrees[ROLL],
	                             -sensors.attitude500Hz[ROLL] * mechanical2electricalDegrees[ROLL],
	                             dt, holdIntegrators, &eepromConfig.PID[ROLL_PID]);

	    outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL];

	    if (outputRate[ROLL] > eepromConfig.rateLimit)
	        pidCmd[ROLL] = pidCmdPrev[ROLL] + eepromConfig.rateLimit;

	    if (outputRate[ROLL] < -eepromConfig.rateLimit)
	        pidCmd[ROLL] = pidCmdPrev[ROLL] - eepromConfig.rateLimit;

	    pidCmdPrev[ROLL] = pidCmd[ROLL];

	    setRollMotor(pidCmd[ROLL], (int)eepromConfig.rollPower);
    }

    ///////////////////////////////////

    if (eepromConfig.pitchEnabled == true)
    {
        pidCmd[PITCH] = updatePID(pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH],
                                  sensors.attitude500Hz[PITCH] * mechanical2electricalDegrees[PITCH],
                                  dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);

	    outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];

	    if (outputRate[PITCH] > eepromConfig.rateLimit)
	        pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;

	    if (outputRate[PITCH] < -eepromConfig.rateLimit)
	        pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;

	    pidCmdPrev[PITCH] = pidCmd[PITCH];

	    setPitchMotor(pidCmd[PITCH], (int)eepromConfig.pitchPower);
    }

    ///////////////////////////////////

    if (eepromConfig.yawEnabled == true)
    {
        if (eepromConfig.yawAutoPanEnabled == true)
            yawCmd = autoPan(pidCmd[YAW], yawCmd * mechanical2electricalDegrees[YAW]) * electrical2mechanicalDegrees[YAW];
        else
            yawCmd = -pointingCmd[YAW];

        pidCmd[YAW] = updatePID( yawCmd * mechanical2electricalDegrees[YAW],
                                sensors.attitude500Hz[YAW] * mechanical2electricalDegrees[YAW],
                                dt, holdIntegrators, &eepromConfig.PID[YAW_PID]);

	    outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];

	    if (outputRate[YAW] > eepromConfig.rateLimit)
	        pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;

	    if (outputRate[YAW] < -eepromConfig.rateLimit)
	        pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;

	    pidCmdPrev[YAW] = pidCmd[YAW];

        setYawMotor(pidCmd[YAW], (int)eepromConfig.yawPower);
    }

    ///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////
