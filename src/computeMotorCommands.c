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

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

float mechanical2electricalDegrees[3] = { 1.0f, 1.0f, 1.0f };
float electrical2mechanicalDegrees[3] = { 1.0f, 1.0f, 1.0f };

float outputRate[3];

float pidCmd[3];

float pidCmdPrev[3] = { 0.0f, 0.0f, 0.0f };

#define AP_DEADBAND      2.00f  // in radians with respect to one motor pole, actual angle is (DEADBAND / numberPoles) * R2D
#define MOTORPOS2SETPNT  0.35f  // scaling factor for how fast it should move
#define AUTOPANSMOOTH   40.00f

float centerPoint[3] = { 0.0f, 0.0f, 0.0f };
float stepSmooth[3]  = { 0.0f, 0.0f, 0.0f };
float step[3]        = { 0.0f, 0.0f, 0.0f };

uint8_t autoPanFirstPass[3] = { true, true, true };

///////////////////////////////////////////////////////////////////////////////
// Roll AutoPan
///////////////////////////////////////////////////////////////////////////////

float rollAutoPan(float motorPos, float setpoint)
{
    if (autoPanFirstPass[ROLL] == true)
    {
		centerPoint[ROLL] = motorPos;
        autoPanFirstPass[ROLL] = false;
	}

    if (motorPos < centerPoint[ROLL] - AP_DEADBAND)
    {
        centerPoint[ROLL] = AP_DEADBAND;
        step[ROLL] = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else if (motorPos > centerPoint[ROLL] + AP_DEADBAND)
    {
        centerPoint[ROLL] = -AP_DEADBAND;
        step[ROLL] = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else
    {
        step[ROLL] = 0.0f;
        centerPoint[ROLL] = 0.0f;
    }

    stepSmooth[ROLL] = (stepSmooth[ROLL] * (AUTOPANSMOOTH - 1.0f) + step[ROLL]) / AUTOPANSMOOTH;

    return (setpoint -= stepSmooth[ROLL]);
}

///////////////////////////////////////////////////////////////////////////////
// Pitch AutoPan
///////////////////////////////////////////////////////////////////////////////

float pitchAutoPan(float motorPos, float setpoint)
{
    if (autoPanFirstPass[PITCH] == true)
    {
		centerPoint[PITCH] = motorPos;
        autoPanFirstPass[PITCH] = false;
	}

    if (motorPos < centerPoint[PITCH] - AP_DEADBAND)
    {
        centerPoint[PITCH] = AP_DEADBAND;
        step[PITCH] = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else if (motorPos > centerPoint[PITCH] + AP_DEADBAND)
    {
        centerPoint[PITCH] = -AP_DEADBAND;
        step[PITCH] = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else
    {
        step[PITCH] = 0.0f;
        centerPoint[PITCH] = 0.0f;
    }

    stepSmooth[PITCH] = (stepSmooth[PITCH] * (AUTOPANSMOOTH - 1.0f) + step[PITCH]) / AUTOPANSMOOTH;

    return (setpoint -= stepSmooth[PITCH]);
}

///////////////////////////////////////////////////////////////////////////////
// Yaw AutoPan
///////////////////////////////////////////////////////////////////////////////

float yawAutoPan(float motorPos, float setpoint)
{
    if (autoPanFirstPass[YAW] == true)
    {
		centerPoint[YAW] = motorPos;
        autoPanFirstPass[YAW] = false;
	}

    if (motorPos < centerPoint[YAW] - AP_DEADBAND)
    {
        centerPoint[YAW] = AP_DEADBAND;
        step[YAW] = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else if (motorPos > centerPoint[YAW] + AP_DEADBAND)
    {
        centerPoint[YAW] = -AP_DEADBAND;
        step[YAW] = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else
    {
        step[YAW] = 0.0f;
        centerPoint[YAW] = 0.0f;
    }

    stepSmooth[YAW] = (stepSmooth[YAW] * (AUTOPANSMOOTH - 1.0f) + step[YAW]) / AUTOPANSMOOTH;

    return (setpoint -= stepSmooth[YAW]);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Motor Commands
///////////////////////////////////////////////////////////////////////////////

void computeMotorCommands(float dt)
{
	float rollCmdDiv2, pitchCmdDiv2, yawCmdDiv2;

	float cosRollCmdDiv2,  sinRollCmdDiv2;
	float cosPitchCmdDiv2, sinPitchCmdDiv2;
	float cosYawCmdDiv2,   sinYawCmdDiv2;

	float qRef[4];

	float qMeasConjugate[4];

	float qError[4];

	float normR;

	float qError0Squared, qError1Squared, qError2Squared, qError3Squared;

	float axisError[3];

	holdIntegrators = false;

	///////////////////////////////////

	rollCmdDiv2  = pointingCmd[ROLL ] / 2.0f;
	pitchCmdDiv2 = pointingCmd[PITCH] / 2.0f;
	yawCmdDiv2   = pointingCmd[YAW  ] / 2.0f;

	cosRollCmdDiv2  = cosf(rollCmdDiv2);
	sinRollCmdDiv2  = sinf(rollCmdDiv2);

	cosPitchCmdDiv2 = cosf(pitchCmdDiv2);
	sinPitchCmdDiv2 = sinf(pitchCmdDiv2);

	cosYawCmdDiv2   = cosf(yawCmdDiv2);
	sinYawCmdDiv2   = sinf(yawCmdDiv2);

	qRef[0] = ( cosRollCmdDiv2 * cosPitchCmdDiv2 * cosYawCmdDiv2 ) +
    		  ( sinRollCmdDiv2 * sinPitchCmdDiv2 * sinYawCmdDiv2 );

    qRef[1] = ( sinRollCmdDiv2 * cosPitchCmdDiv2 * cosYawCmdDiv2 ) -
  		      ( cosRollCmdDiv2 * sinPitchCmdDiv2 * sinYawCmdDiv2 );

    qRef[2] = ( cosRollCmdDiv2 * sinPitchCmdDiv2 * cosYawCmdDiv2 ) +
  		      ( sinRollCmdDiv2 * cosPitchCmdDiv2 * sinYawCmdDiv2 );

    qRef[3] = ( cosRollCmdDiv2 * cosPitchCmdDiv2 * sinYawCmdDiv2 ) -
  		      ( sinRollCmdDiv2 * sinPitchCmdDiv2 * cosYawCmdDiv2 );

    qMeasConjugate[0] =  qMeas[0];

    qMeasConjugate[1] = -qMeas[1];

    qMeasConjugate[2] = -qMeas[2];

    qMeasConjugate[3] = -qMeas[3];

    qError[0] = qRef[0] * qMeasConjugate[0] - qRef[1] * qMeasConjugate[1] -
    		    qRef[2] * qMeasConjugate[2] - qRef[3] * qMeasConjugate[3];

    qError[1] = qRef[0] * qMeasConjugate[1] + qRef[1] * qMeasConjugate[0] +
    		    qRef[2] * qMeasConjugate[3] - qRef[3] * qMeasConjugate[2];

    qError[2] = qRef[0] * qMeasConjugate[2] - qRef[1] * qMeasConjugate[3] +
    		    qRef[2] * qMeasConjugate[0] + qRef[3] * qMeasConjugate[1];

    qError[3] = qRef[0] * qMeasConjugate[3] + qRef[1] * qMeasConjugate[2] -
    		    qRef[2] * qMeasConjugate[1] + qRef[3] * qMeasConjugate[0];

    // normalize quaternion
    normR = 1.0f / sqrt(qError[0] * qError[0] + qError[1] * qError[1] + qError[2] * qError[2] + qError[3] * qError[3]);

    qError[0] *= normR;
    qError[1] *= normR;
    qError[2] *= normR;
    qError[3] *= normR;

    qError0Squared = qError[0] * qError[0];

    qError1Squared = qError[1] * qError[1];

    qError2Squared = qError[2] * qError[2];

    qError3Squared = qError[3] * qError[3];

    axisError[ROLL ] = atan2f(2.0f * (qError[0] * qError[1] + qError[2] * qError[3]),
    		                  qError0Squared - qError1Squared - qError2Squared + qError3Squared);

    axisError[PITCH] = asinf(2.0f * (qError[0] * qError[2] - qError[3] * qError[1]));

    axisError[YAW  ] = atan2f(2.0f * (qError[0] * qError[3] + qError[1] * qError[2]),
    		                  qError0Squared + qError1Squared - qError2Squared - qError3Squared);

    ///////////////////////////////////

	if (eepromConfig.rollEnabled == true)
	{
        pidCmd[ROLL] = updatePID(0.0f, axisError[ROLL] * mechanical2electricalDegrees[ROLL],
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
        pidCmd[PITCH] = updatePID(axisError[PITCH] * mechanical2electricalDegrees[PITCH], 0.0f,
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
        pidCmd[YAW] = updatePID(axisError[YAW] * mechanical2electricalDegrees[YAW], 0.0f,
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
