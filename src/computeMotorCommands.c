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

float pitchSetpoint = 0.0f, pitchErrorLast, pitchAngleCorrection;
float rollSetpoint = 0.0f,  rollErrorLast,  rollAngleCorrection;
float yawSetpoint = 0.0f,   yawErrorLast,   yawAngleCorrection;

float pidCmd[3];

float ADC1Ch13_yaw;

static float rollRCOffset = 0.0f, pitchRCOffset = 0.0f, yawRCOffset = 0.0f;

float accAngleSmooth[3];

float Step[NUMAXIS]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[NUMAXIS] = {0.0f, 0.0f, 0.0f};

///////////////////////////////////////////////////////////////////////////////
//  Limit the Pitch Angle
///////////////////////////////////////////////////////////////////////////////

float limitPitch(float step, float pitch)
{
    if (pitch < PITCH_UP_LIMIT && step > 0.0f)
    {
        step = 0.0f;
    }

    if (pitch > PITCH_DOWN_LIMIT && step < 0.0f)
    {
        step = 0.0f;
    }

    return step;
}

///////////////////////////////////////////////////////////////////////////////
// Compute Motor Commands
///////////////////////////////////////////////////////////////////////////////

void computeMotorCommands(float dt)
{
	holdIntegrators = false;

	///////////////////////////////////

    // if we enable RC control
    // HJI if (configData[9] == '1')
    // HJI {
    // HJI     // Get the RX values and Averages
    // HJI     Get_RC_Step(Step, RCSmooth);                                  // Get RC movement on all three AXIS
    // HJI     Step[PITCH] = limitPitch(Step[PITCH], sensors.attitude500Hz[PITCH]);  // limit pitch to defined limits in header
    // HJI }

	///////////////////////////////////

	if (eepromConfig.rollEnabled == true)
	{
	    //roll_setpoint += Step[ROLL];
	    rollRCOffset  += Step[ROLL] / 1000.0f;

	    // include the config roll offset which is scaled to 0 = -10.0 degrees, 100 = 0.0 degrees, and 200 = 10.0 degrees
	    // HJI roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset + ((configData[11] - 100) / 10.0) / R2D) * 50.0, -CORRECTION_STEP, CORRECTION_STEP);
	    rollAngleCorrection = constrain((sensors.attitude500Hz[ROLL] + rollRCOffset) * 50.0f, -CORRECTION_STEP, CORRECTION_STEP);
	    rollSetpoint += rollAngleCorrection; //Roll return to zero after collision

	    pidCmd[ROLL] = updatePID(rollSetpoint + sensors.attitude500Hz[ROLL] * 1000.0f, 0.0f, dt, holdIntegrators, &eepromConfig.PID[ROLL_PID]);

	    setRollMotor(pidCmd[ROLL], (int)eepromConfig.rollPower);
    }

    ///////////////////////////////////

    if (eepromConfig.pitchEnabled == true)
    {
        //pitch_setpoint += Step[PITCH];
        pitchRCOffset  += Step[PITCH] / 1000.0f;

        pitchAngleCorrection = constrain((sensors.attitude500Hz[PITCH] + pitchRCOffset) * 50.0f, -CORRECTION_STEP, CORRECTION_STEP);
        pitchSetpoint += pitchAngleCorrection; // Pitch return to zero after collision

        pidCmd[PITCH] = updatePID(pitchSetpoint + sensors.attitude500Hz[PITCH] * 1000.0f, 0.0f, dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);

        setPitchMotor(pidCmd[PITCH], (int)eepromConfig.pitchPower);
    }

    ///////////////////////////////////

    // if we enabled AutoPan on Yaw
    // HJI if (configData[10] == '0')
    // HJI {
    // HJI     ADC1Ch13_yaw = ((ADC1Ch13_yaw * 99.0) + ((float)(readADC1(13) - 2000) / 4000.0)) / 100.0;  // Average ADC value
    // HJI }

    // Yaw Adjustments
    yawSetpoint += Step[YAW];
    yawRCOffset  += Step[YAW] / 1000.0f;

    // if AutoPan is enabled
    // HJI if (configData[10] == '0')
    // HJI {
    // HJI     CameraOrient[YAW] = CameraOrient[YAW] + 0.01 * (ADC1Ch13_yaw - CameraOrient[YAW]);
    // HJI }

#if 0
    yaw_angle_correction = constrain((CameraOrient[YAW] + yawRCOffset) * 50.0, -CORRECTION_STEP, CORRECTION_STEP);
    yaw_setpoint += yaw_angle_correction; // Yaw return to zero after collision
#endif

    pidCmd[YAW] = updatePID(yawSetpoint + sensors.attitude500Hz[YAW] * 1000.0f, 0.0f, dt, holdIntegrators, &eepromConfig.PID[YAW_PID]);

    setYawMotor(pidCmd[YAW], (int)eepromConfig.yawPower);

    ///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////
