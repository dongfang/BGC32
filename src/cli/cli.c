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

uint8_t cliBusy = false;

static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;

///////////////////////////////////////

uint8_t gimbalStateEnabled = true;

uint8_t savedRollState;
uint8_t savedPitchState;
uint8_t savedYawState;

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

uint8_t readStringCLI(uint8_t *data)
{
    int index             = 0;
    int timeout           = 0;
    uint8_t elementCount  = 0;
    uint8_t validEnd      = false;
    uint8_t stringIsValid = true;
    uint8_t *stringEnd    = NULL;

    do
    {
        // are there characters, if not wait 10ms and increment timeout counter
        if (cliAvailable() == 0)
        {
            delay(10);
            timeout++;
        }
        else
        {
            // chars available, let's read them and build up a string
            //  we'll check to see if it ever has a character other than those listed
            // below and set a flag to use further down
            cliRead(&data[index], 1);

            if (!isdigit(data[index]) && data[index] != ';' && data[index] != '\r' && data[index] != '\n' && data[index] != '\b' && data[index] != '.' && data[index] != '-')
            {
                stringIsValid = false;
            }

            // how many elements are there in the string
            if (data[index] == ';')
                elementCount++;

            // have to handle the first char being a backspace
            // and how to handle backspace in general
            if (data[index] == '\b' && index > 0)
            {
                // decrement the index, but let make sure and keep the index positive only
                if (index-- < 0)
                    index = 0;
            }
            else
            {
                // valid char so increment the pointer
                index++;
            }

            // ok, we got an end of line character, now we have to handle crlf and lf only
            if (data[index - 1] == '\n')
            {
                if (index > 1)
                {
                    if (data[index - 2] == '\r')
                    {
                        // null terminate the string
                        data[index - 2] = '\0';
                        // we save the last char position in case we want to use it later
                        stringEnd = &data[index - 2];
                    }
                    else
                    {
                        // null terminate the string
                        data[index - 1] = '\0';
                        // we save the last char position in case we want to use it later
                        stringEnd = &data[index - 1];
                    }
                }
                else
                {
                    // null terminate the string
                    data[index - 1] = '\0';
                    // we save the last char position in case we want to use it later
                    stringEnd = &data[index - 1];
                }

                // we got a valid of line so let's break from the do/while early
                validEnd = true;
            }

            // each time we get a char let's clear the timeout counter
            timeout = 0;
        }
    }
    while ((index == 0 || !validEnd) && (timeout < 1500)); // we'll stay in the do/while until we either get a valid end or we timeout (15 second wait)

    // do we have a string with all valid characters
    // did it end correctly (to be used if we want to force the ';' final character)
    // and is the string not a null string
    if (stringIsValid && (stringEnd && (stringEnd - data > 0)) )
    {
        stringIsValid = elementCount;
//        cliPrintF("string length = %d\n", stringEnd - data);
//        cliPrintF("element count = %d\n", elementCount);
//        cliPrintF("last char [%d]\n", *stringEnd);
    }
    else
    {
        stringIsValid = false;
    }

    return stringIsValid;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    uint8_t    data[13] = "";

    do
    {
        if (cliAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            //data[index] = cliRead();
            cliRead(&data[index], 1);
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index - 1] != ';') && (timeout < 5) && (index < sizeof(data) - 1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from CLI
///////////////////////////////////////////////////////////////////////////////

uint8_t readCliPID(unsigned char PIDid, uint8_t *tempString)
{
    struct PIDdata *pid = &eepromConfig.PID[PIDid];
    char *token;
    char *state;
    uint8_t readSuccess = true;

    if (readStringCLI(tempString) == 6)
    {
        token = strtok_r((char *)tempString, ";", &state);
        if (token != NULL)
            pid->B              = stringToFloat((uint8_t *)token);

        token = strtok_r(NULL, ";", &state);
        if (token != NULL)
            pid->P              = stringToFloat((uint8_t *)token);

        token = strtok_r(NULL, ";", &state);
        if (token != NULL)
            pid->I              = stringToFloat((uint8_t *)token);

        token = strtok_r(NULL, ";", &state);
        if (token != NULL)
            pid->D              = stringToFloat((uint8_t *)token);

        token = strtok_r(NULL, ";", &state);
        if (token != NULL)
            pid->windupGuard    = stringToFloat((uint8_t *)token);

        pid->iTerm          = 0.0f;
        pid->lastDcalcValue = 0.0f;
        pid->lastDterm      = 0.0f;
        pid->lastLastDterm  = 0.0f;

        token = strtok_r(NULL, ";", &state);
        if (token != NULL)
            pid->dErrorCalc     = (uint8_t)stringToFloat((uint8_t *)token);
    }
    else
    {
//        cliPrintF("\nInput Error\n");
        readSuccess = false;
    }

    return readSuccess;
}

///////////////////////////////////////////////////////////////////////////////
// Disable Gimbal
///////////////////////////////////////////////////////////////////////////////

void disableGimbal(void)
{
    savedRollState  = eepromConfig.rollEnabled;
    savedPitchState = eepromConfig.pitchEnabled;
    savedYawState   = eepromConfig.yawEnabled;

    eepromConfig.rollEnabled  = false;
    eepromConfig.pitchEnabled = false;
    eepromConfig.yawEnabled   = false;

    pwmMotorDriverInit();

    cliPrintF("\nGimbal Disabled, CLI active....\n");
}

///////////////////////////////////////////////////////////////////////////////
// Enable Gimbal
///////////////////////////////////////////////////////////////////////////////

void enableGimbal(void)
{
    eepromConfig.rollEnabled  = savedRollState;
    eepromConfig.pitchEnabled = savedPitchState;
    eepromConfig.yawEnabled   = savedYawState;

    pwmMotorDriverInit();

    cliPrintF("\nGimbal Enabled, CLI inactive....\n");
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
    // string used for cli commands, needs to be as long as need, but
    // not too long
    uint8_t tempString[40] = "";
    char *token;
    char *state;

    if ((cliAvailable() && !validCliCommand))
    {
        cliQuery = getChar();

        // check for a 'Z' command to enable/disable the motors
        if (cliQuery == 'Z')
        {
            if (gimbalStateEnabled == true)
            {
                disableGimbal();

                gimbalStateEnabled = false;
            }
            else
            {
                enableGimbal();

                gimbalStateEnabled = true;
            }
        }
    }

    validCliCommand = false;

    // CLI is only available when the gimbal/motors are disabled
    if (!gimbalStateEnabled && (cliQuery != 'Z'))
    {
        switch (cliQuery)
        {
                ///////////////////////////////

            case 'a': // Rate PIDs
                cliPrintF("\nRoll Rate PID:  %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %s\n", eepromConfig.PID[ROLL_PID].B,
                                                                                       eepromConfig.PID[ROLL_PID].P,
                                                                                       eepromConfig.PID[ROLL_PID].I,
                                                                                       eepromConfig.PID[ROLL_PID].D,
                                                                                       eepromConfig.PID[ROLL_PID].windupGuard,
                                                                                       eepromConfig.PID[ROLL_PID].dErrorCalc ? "Error" : "State");

                cliPrintF("Pitch Rate PID: %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %s\n",   eepromConfig.PID[PITCH_PID].B,
                                                                                       eepromConfig.PID[PITCH_PID].P,
                                                                                       eepromConfig.PID[PITCH_PID].I,
                                                                                       eepromConfig.PID[PITCH_PID].D,
                                                                                       eepromConfig.PID[PITCH_PID].windupGuard,
                                                                                       eepromConfig.PID[PITCH_PID].dErrorCalc ? "Error" : "State");

                cliPrintF("Yaw Rate PID:   %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %s\n",   eepromConfig.PID[YAW_PID].B,
                                                                                       eepromConfig.PID[YAW_PID].P,
                                                                                       eepromConfig.PID[YAW_PID].I,
                                                                                       eepromConfig.PID[YAW_PID].D,
                                                                                       eepromConfig.PID[YAW_PID].windupGuard,
                                                                                       eepromConfig.PID[YAW_PID].dErrorCalc ? "Error" : "State");
                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'b': // Loop Delta Times
                cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", deltaTime1000Hz,
                                                                        deltaTime500Hz,
                                                                        deltaTime100Hz,
                                                                        deltaTime50Hz,
                                                                        deltaTime10Hz,
                                                                        deltaTime5Hz,
                                                                        deltaTime1Hz);
                break;

                ///////////////////////////////

            case 'c': // Loop Execution Times
                cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", executionTime1000Hz,
                                                                              executionTime500Hz,
                                                                              executionTime100Hz,
                                                                              executionTime50Hz,
                                                                              executionTime10Hz,
                                                                              executionTime5Hz,
                                                                              executionTime1Hz,
                                                                              i2cGetErrorCounter());
                break;

                ///////////////////////////////

            case 'd': // RC Parameters
                cliPrintF("\n       RC Response    Max Rate    Left/Down Limit    Right/Up Limit\n");

                cliPrintF("Roll:     %s         %4.1f", eepromConfig.rollRateCmdInput  ? "Rate" : "Att ", eepromConfig.gimbalRollRate * R2D);
                cliPrintF("          %5.1f              %5.1f\n", eepromConfig.gimbalRollLeftLimit * R2D, eepromConfig.gimbalRollRightLimit * R2D);

                cliPrintF("Pitch:    %s         %4.1f", eepromConfig.pitchRateCmdInput ? "Rate" : "Att ", eepromConfig.gimbalPitchRate * R2D);
                cliPrintF("          %5.1f              %5.1f\n", eepromConfig.gimbalPitchDownLimit * R2D, eepromConfig.gimbalPitchUpLimit * R2D);

                cliPrintF("Yaw:      %s         %4.1f", eepromConfig.yawRateCmdInput   ? "Rate" : "Att ", eepromConfig.gimbalYawRate * R2D);
                cliPrintF("          %5.1f              %5.1f\n", eepromConfig.gimbalYawLeftLimit * R2D, eepromConfig.gimbalYawRightLimit * R2D);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'e': // 500 Hz Accels
                cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
                                                   sensors.accel500Hz[YAXIS],
                                                   sensors.accel500Hz[ZAXIS]);
                break;

                ///////////////////////////////

            case 'f': // 500 Hz Gyros
                cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ] * R2D,
                                                          sensors.gyro500Hz[PITCH] * R2D,
                                                          sensors.gyro500Hz[YAW  ] * R2D,
                                                          mpu6050Temperature);
                 break;

                ///////////////////////////////

            case 'g': // 10 Hz Mag Data
                cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.mag10Hz[XAXIS],
                                                   sensors.mag10Hz[YAXIS],
                                                   sensors.mag10Hz[ZAXIS]);
                break;

                ///////////////////////////////

            case 'h': // Attitudes
                cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.margAttitude500Hz[ROLL]  * R2D,
                                                   sensors.margAttitude500Hz[PITCH] * R2D,
                                                   sensors.margAttitude500Hz[YAW]   * R2D);

                break;

                ///////////////////////////////

            case 'i': // Gimbal Axis Enable Flags
                cliPrintF("\nGimbal Roll Axis:  %s\n", savedRollState  ? "Enabled" : "Disabled");
                cliPrintF("Gimbal Pitch Axis: %s\n",   savedPitchState ? "Enabled" : "Disabled");
                cliPrintF("Gimbal Yaw Axis:   %s\n",   savedYawState   ? "Enabled" : "Disabled");

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'j': // Gimbal Axis Power Levels
                cliPrintF("\nGimbal Roll Axis Power level:  %4.1f\n", eepromConfig.rollPower);
                cliPrintF("Gimbal Pitch Axis Power level: %4.1f\n",   eepromConfig.pitchPower);
                cliPrintF("Gimbal Yaw Axis Power level:   %4.1f\n",   eepromConfig.yawPower);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'k': // Gimbal Rate Limit
                cliPrintF("\nGimbal Rate Limit: %7.3f\n", eepromConfig.rateLimit * R2D);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'l': // Gimbal IMU Orientation
                cliPrintF("\nGimbal IMU Orientation: %1d\n", eepromConfig.imuOrientation);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'm': // Test Phase Value
                cliPrintF("\nTest Phase Value: %6.2f\n", testPhase * R2D);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'n': // Test Phase Delta
                cliPrintF("\nTest Phase Delta: %6.2f\n", testPhaseDelta * R2D);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'o': // Motor Poles
                cliPrintF("\nRoll Motor Poles:  %3.0f \n", eepromConfig.rollMotorPoles);
                cliPrintF("Pitch Motor Poles: %3.0f \n",   eepromConfig.pitchMotorPoles);
                cliPrintF("Yaw Motor Poles:   %3.0f \n",   eepromConfig.yawMotorPoles);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'p': // Counters
                cliPrintF("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d\n",
                          minCnt[ROLL], minCnt[PITCH], minCnt[YAW],
                          maxCnt[ROLL], maxCnt[PITCH], maxCnt[YAW],
                          irqCnt[ROLL], irqCnt[PITCH], irqCnt[YAW]);
                break;

                ///////////////////////////////

            case 'q': // Filter Time Constants
                cliPrintF("\n         Accel TC       Rate Cmd TC    Att Cmd TC\n");

                cliPrintF("Roll/Y:    %5.2f           %5.2f          %5.2f\n", eepromConfig.accelY500HzLowPassTau,
                                                                               eepromConfig.rollRatePointingCmd50HzLowPassTau,
                                                                               eepromConfig.rollAttPointingCmd50HzLowPassTau);

                cliPrintF("Pitch/X:   %5.2f           %5.2f          %5.2f\n", eepromConfig.accelX500HzLowPassTau,
                                                                               eepromConfig.pitchRatePointingCmd50HzLowPassTau,
                                                                               eepromConfig.pitchAttPointingCmd50HzLowPassTau);

                cliPrintF("Yaw/Z:     %5.2f           %5.2f          %5.2f\n", eepromConfig.accelZ500HzLowPassTau,
                                                                               eepromConfig.yawRatePointingCmd50HzLowPassTau,
                                                                               eepromConfig.yawAttPointingCmd50HzLowPassTau);
                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 's': // Raw Receiver Commands
                cliPrintF("%4i, ", rxRead(ROLL));
                cliPrintF("%4i, ", rxRead(PITCH));
                cliPrintF("%4i\n", rxRead(YAW));
                break;

                ///////////////////////////////

            case 't': // Pointing Commands
                cliPrintF("%8.2f, ", pointingCmd[ROLL]  * R2D);
                cliPrintF("%8.2f, ", pointingCmd[PITCH] * R2D);
                cliPrintF("%8.2f\n", pointingCmd[YAW]   * R2D);
                break;

                ///////////////////////////////

            case 'u': // PID Outputs
                cliPrintF("%12.4f, %12.4f, %12.4f\n", pidCmd[ROLL],
                                                      pidCmd[PITCH],
                                                      pidCmd[YAW]);
                break;

                ///////////////////////////////

            case 'v': // Version

                cliPrintF("BGC32 Firmware V%s, Build Date " __DATE__ " "__TIME__" \n", __BGC32_VERSION);

                cliQuery = 'x';
                break;

                ///////////////////////////////

            case 'x':
                break;

                ///////////////////////////////

            case 'y':  // AutoPan Enable Status
                cliPrintF("\nRoll AutoPan:  (Not Implemented)  %s\n", eepromConfig.rollAutoPanEnabled  ? "Enabled" : "Disabled");
                cliPrintF("Pitch AutoPan: (Not Implemented)  %s\n",   eepromConfig.pitchAutoPanEnabled ? "Enabled" : "Disabled");
                cliPrintF("Yaw AutoPan:                      %s\n",   eepromConfig.yawAutoPanEnabled   ? "Enabled" : "Disabled");

                cliQuery = 'x';
                break;

                ///////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////

                ///////////////////////////////

            case 'A': // Read Roll PID Values
                if (readCliPID(ROLL_PID, tempString))
                {
                    cliPrintF("\nRoll Rate PID Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'a';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'B': // Read Pitch PID Values
                if (readCliPID(PITCH_PID, tempString))
                {
                    cliPrintF("\nPitch Rate PID Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'a';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'C': // Read Yaw PID Values
                if (readCliPID(YAW_PID, tempString))
                {
                    cliPrintF("\nYaw Rate PID Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'a';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'D': // Read Roll RC Parameters
                if (readStringCLI(tempString) == 4)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.rollRateCmdInput     = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalRollRate       = stringToFloat((uint8_t *)token) * D2R;

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalRollLeftLimit  = stringToFloat((uint8_t *)token) * D2R;

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalRollRightLimit = stringToFloat((uint8_t *)token) * D2R;

                    cliPrintF("\nRoll RC Parameters Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'd';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'E': // Read Pitch RC Parameters
                if (readStringCLI(tempString) == 4)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.pitchRateCmdInput     = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalPitchRate       = stringToFloat((uint8_t *)token) * D2R;

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalPitchDownLimit  = stringToFloat((uint8_t *)token) * D2R;

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalPitchUpLimit    = stringToFloat((uint8_t *)token) * D2R;

                    cliPrintF("\nPitch RC Parameters Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'd';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'F': // Read Yaw RC Parameters
                if (readStringCLI(tempString) == 4)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.yawRateCmdInput     = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalYawRate       = stringToFloat((uint8_t *)token) * D2R;

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalYawLeftLimit  = stringToFloat((uint8_t *)token) * D2R;

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.gimbalYawRightLimit = stringToFloat((uint8_t *)token) * D2R;

                    cliPrintF("\nYaw RC Parameters Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'd';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'I': // Read Gimbal Axis Enable Flags
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        savedRollState  = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        savedPitchState = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        savedYawState   = (uint8_t)stringToFloat((uint8_t *)token);

                    cliPrintF("\nGimbal Axis Enable Flags Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'i';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'J': // Read Gimbal Axis Power Levels
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.rollPower  = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.pitchPower = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.yawPower   = stringToFloat((uint8_t *)token);

                    cliPrintF("\nGimbal Axis Power Levels Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'j';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'K': // Read Gimbal Rate Limit
                if (readStringCLI(tempString) == 1)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.rateLimit = stringToFloat((uint8_t *)token) * D2R;

                    cliPrintF("\nGimbal Rate Limit Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'k';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'L': // Read Gimbal IMU Orientation
                if (readStringCLI(tempString) == 1)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.imuOrientation = (uint8_t)stringToFloat((uint8_t *)token);

                    cliPrintF("\nGimbal IMU Orientation Received....\n");

                    orientIMU();
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'l';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'M': // Read Test Phase
                if (readStringCLI(tempString) == 1)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        testPhase = stringToFloat((uint8_t *)token) * D2R;

                    cliPrintF("\nTest Phase Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'm';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'N': // Read Test Phase Delta
                if (readStringCLI(tempString) == 1)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        testPhaseDelta = stringToFloat((uint8_t *)token) * D2R;

                    cliPrintF("\nTest Phase Delta Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'n';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'O': // Set Motor Poles
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.rollMotorPoles  = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.pitchMotorPoles = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.yawMotorPoles   = stringToFloat((uint8_t *)token);

                    cliPrintF("\nMotor Pole Counts Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'o';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'P': // Sensor CLI
                sensorCLI(tempString);

                cliQuery = 'x';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'Q': // Read Roll Filter Time Constants
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.accelY500HzLowPassTau             = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.rollRatePointingCmd50HzLowPassTau = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.rollAttPointingCmd50HzLowPassTau  = stringToFloat((uint8_t *)token);

                    initFirstOrderFilter();
                    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS ].previousInput  = sensors.accel500Hz[YAXIS];
                    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS ].previousOutput = sensors.accel500Hz[YAXIS];

                    cliPrintF("\nRoll Filter Time Constants Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'q';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'R': // Reset to Bootloader
                cliPrintF("\nEntering Bootloader....\n\n");
                delay(1000);
                bootloader();
                break;

                ///////////////////////////////

            case 'S': // Reset System
                cliPrintF("\nSystem Rebooting....\n\n");
                delay(1000);
                reboot();
                break;

                ///////////////////////////////

            case 'T': // Read Pitch Filter Time Constants
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.accelX500HzLowPassTau              = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.pitchRatePointingCmd50HzLowPassTau = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.pitchAttPointingCmd50HzLowPassTau  = stringToFloat((uint8_t *)token);

                    initFirstOrderFilter();
                    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousInput  = sensors.accel500Hz[XAXIS];
                    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousOutput = sensors.accel500Hz[XAXIS];

                    cliPrintF("\nPitch Filter Time Constants Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'q';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'U': // Read Yaw Filter Time Constants
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.accelZ500HzLowPassTau            = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.yawRatePointingCmd50HzLowPassTau = stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.yawAttPointingCmd50HzLowPassTau  = stringToFloat((uint8_t *)token);

                    initFirstOrderFilter();
                    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS  ].previousInput  = sensors.accel500Hz[ZAXIS];
                    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS  ].previousOutput = sensors.accel500Hz[ZAXIS];

                    cliPrintF("\nYaw Filter Time Constants Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'q';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'V': // Reset EEPROM Parameters
                cliPrintF("\nEEPROM Parameters Reset....\n");
                checkFirstTime(true);
                cliPrintF("\nSystem Rebooting....\n\n");
                delay(1000);
                reboot();
                break;

                ///////////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrintF("\nWriting EEPROM Parameters....\n");
                writeEEPROM();

                cliQuery = 'x';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'X': // Not Used
                cliQuery = 'x';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case 'Y': // Read AutoPan Enable Flags
                if (readStringCLI(tempString) == 3)
                {
                    token = strtok_r((char *)tempString, ";", &state);
                    if (token != NULL)
                        eepromConfig.rollAutoPanEnabled  = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.pitchAutoPanEnabled = (uint8_t)stringToFloat((uint8_t *)token);

                    token = strtok_r(NULL, ";", &state);
                    if (token != NULL)
                        eepromConfig.yawAutoPanEnabled   = (uint8_t)stringToFloat((uint8_t *)token);

                    eepromConfig.rollAutoPanEnabled  = false;  // HJI Function not implemented yet
                    eepromConfig.pitchAutoPanEnabled = false;  // HJI Function not implemented yet

                    cliPrintF("\nAutoPan Enable Flags Received....\n");
                }
                else
                {
                    cliPrintF("\nInput Error\n");
                }

                cliQuery = 'y';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case '+': // Increment Test Phase
                testPhase += testPhaseDelta;

                cliQuery = 'm';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case '-': // Decrement Test Phase
                testPhase -= testPhaseDelta;

                cliQuery = 'm';
                validCliCommand = true;
                break;

                ///////////////////////////////

            case '?': // Command Summary
                cliBusy = true;

                cliPrintF("\n");
                cliPrintF("'a' Rate PIDs                      'A' Set Roll Rate PID Data       AB;P;I;D;windupGuard;dErrorCalc\n");
                cliPrintF("'b' Loop Delta Times               'B' Set Pitch Rate PID Data      BB;P;I;D;windupGuard;dErrorCalc\n");
                cliPrintF("'c' Loop Execution Times           'C' Set Yaw Rate PID Data        CB;P;I;D;windupGuard;dErrorCalc\n");
                cliPrintF("'d' RC Parameters                  'D' Set Roll RC Parameters       DResponse;Rate;Left Limit;Right Limit\n");
                cliPrintF("'e' 500 Hz Accels                  'E' Set Pitch RC Parameters      EResponse;Rate;Down Limit;Up Limit\n");;
                cliPrintF("'f' 500 Hz Gyros                   'F' Set Yaw RC Parameters        FResponse;Rate;Left Limit;Right Limit\n");
                cliPrintF("'g' 10 hz Mag Data                 'G' Not Used\n");
                cliPrintF("'h' Attitudes                      'H' Not Used\n");
                cliPrintF("'i' Gimbal Axis Enable Flags       'I' Set Gimbal Axis Enable Flags IR;P;Y\n");
                cliPrintF("'j' Gimbal Axis Power Settings     'J' Set Gimbal Axis Power Levels JR;P;Y\n");
                cliPrintF("'k' Gimbal Rate Limit              'K' Set Gimbal Rate Limit\n");
                cliPrintF("'l' Gimbal IMU Orientation         'L' Set Gimbal IMU Orientation   LX, X = 1 thru 4\n");
                cliPrintF("\n");

                cliPrintF("Press space bar for more, or enter a command....\n");

                while (cliAvailable() == false);

                cliQuery = getChar();

                if (cliQuery != ' ')
                {
                    validCliCommand = true;
                    cliBusy = false;
                    return;
                }

                cliPrintF("\n");
                cliPrintF("'m' Test Phase                     'M' Set Test Phase\n");
                cliPrintF("'n' Test Phase Delta               'N' Set Test Phase Delta\n");
                cliPrintF("'o' Motor Pole Counts              'O' Set Motor Pole Counts        ORPC;PPC;YPC\n");
                cliPrintF("'p' Counters                       'P' Sensor CLI\n");
                cliPrintF("'q' Filter Time Constants          'Q' Set Roll Filters             QAtt;RateCmd;AttCmd\n");
                cliPrintF("'r' Not Used                       'R' Reset and Enter Bootloader\n");
                cliPrintF("'s' Raw Receiver Commands          'S' Reset\n");
                cliPrintF("'t' Pointing Commands              'T' Set Pitch Filters            TAtt;RateCmd;AttCmd\n");
                cliPrintF("'u' PID Outputs                    'U' Set Yaw Filters              UAtt;RateCmd;AttCmd\n");
                cliPrintF("'v' Version                        'V' Reset EEPROM Parameters\n");
                cliPrintF("'w' Not Used                       'W' Write EEPROM Parameters\n");
                cliPrintF("'x' Terminate CLI Communication    'X' Not Used\n");
                cliPrintF("\n");

                cliPrintF("Press space bar for more, or enter a command....\n");

                while (cliAvailable() == false);

                cliQuery = getChar();

                if (cliQuery != ' ')
                {
                    validCliCommand = true;
                    cliBusy = false;
                    return;
                }

                cliPrintF("\n");
                cliPrintF("'y' AutoPan Enbale Flags           'Y' Set AutoPan Enable Flags     YR;P;Y\n");
                cliPrintF("'z' Not Used                       'Z' Toggle Gimbal Enable/Disable State\n");
                cliPrintF("'+' Increment Test Phase           '?' Command Summary\n");
                cliPrintF("'-' Decrement Test Phase\n");
                cliPrintF("\n");

                cliQuery = 'x';
                cliBusy = false;
                break;

                ///////////////////////////////

            default:
                cliPrintF("\nIgnoring Unknown Command %c (0x%2X)\n", cliQuery, cliQuery);
                cliQuery = 'x';
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
