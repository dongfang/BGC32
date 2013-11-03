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

uint8_t cliBusy = false;

static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if (cliAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < length));

    data[index] = '\0';

    return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if (cliAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from CLI
///////////////////////////////////////////////////////////////////////////////

void readCliPID(unsigned char PIDid)
{
  struct PIDdata* pid = &eepromConfig.PID[PIDid];

  pid->B              = readFloatCLI();
  pid->P              = readFloatCLI();
  pid->I              = readFloatCLI();
  pid->D              = readFloatCLI();
  pid->windupGuard    = readFloatCLI();
  pid->iTerm          = 0.0f;
  pid->lastDcalcValue = 0.0f;
  pid->lastDterm      = 0.0f;
  pid->lastLastDterm  = 0.0f;
  pid->dErrorCalc     =(uint8_t)readFloatCLI();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	if ((cliAvailable() && !validCliCommand))
    	cliQuery = cliRead();

    switch (cliQuery)
    {
        ///////////////////////////////

        case 'a': // Rate PIDs
            cliPrintF("\nRoll Rate PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_PID].B,
                            		                                               eepromConfig.PID[ROLL_PID].P,
                		                                                           eepromConfig.PID[ROLL_PID].I,
                		                                                           eepromConfig.PID[ROLL_PID].D,
                		                                                           eepromConfig.PID[ROLL_PID].windupGuard,
                		                                                           eepromConfig.PID[ROLL_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Pitch Rate PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_PID].B,
                            		                                               eepromConfig.PID[PITCH_PID].P,
                		                                                           eepromConfig.PID[PITCH_PID].I,
                		                                                           eepromConfig.PID[PITCH_PID].D,
                		                                                           eepromConfig.PID[PITCH_PID].windupGuard,
                		                                                           eepromConfig.PID[PITCH_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Yaw Rate PID:   %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[YAW_PID].B,
                             		                                               eepromConfig.PID[YAW_PID].P,
                		                                                           eepromConfig.PID[YAW_PID].I,
                		                                                           eepromConfig.PID[YAW_PID].D,
                		                                                           eepromConfig.PID[YAW_PID].windupGuard,
                		                                                           eepromConfig.PID[YAW_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
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
        	validCliCommand = false;
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
        	validCliCommand = false;
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
          	validCliCommand = false;
          	break;

        ///////////////////////////////

        case 'e': // 500 Hz Accels
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
        			                           sensors.accel500Hz[YAXIS],
        			                           sensors.accel500Hz[ZAXIS]);
        	validCliCommand = false;
            break;

        ///////////////////////////////

        case 'f': // 500 Hz Gyros
        	cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ] * R2D,
        			                                  sensors.gyro500Hz[PITCH] * R2D,
        					                          sensors.gyro500Hz[YAW  ] * R2D,
        					                          mpu6050Temperature);
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'g': // 10 Hz Mag Data
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.mag10Hz[XAXIS],
        			                           sensors.mag10Hz[YAXIS],
        			                           sensors.mag10Hz[ZAXIS]);
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'h': // Attitudes
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ] * R2D,
        	 		                           sensors.attitude500Hz[PITCH] * R2D,
        	 		                           sensors.attitude500Hz[YAW  ] * R2D);
            validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'i': // Gimbal Axis Enable Flags
           	cliPrintF("Gimbal Roll Axis:  %s\n", eepromConfig.rollEnabled  ? "Enabled" : "Disabled");
           	cliPrintF("Gimbal Pitch Axis: %s\n", eepromConfig.pitchEnabled ? "Enabled" : "Disabled");
           	cliPrintF("Gimbal Yaw Axis:   %s\n", eepromConfig.yawEnabled   ? "Enabled" : "Disabled");
           	cliPrintF("Yaw AutoPan:       %s\n", eepromConfig.yawAutoPan   ? "Enabled" : "Disabled");

           	cliQuery = 'x';
          	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'j': // Gimbal Axis Power Levels
           	cliPrintF("Gimbal Roll Axis Power level:  %4.1f\n", eepromConfig.rollPower);
           	cliPrintF("Gimbal Pitch Axis Power level: %4.1f\n", eepromConfig.pitchPower);
           	cliPrintF("Gimbal Yaw Axis Power level:   %4.1f\n", eepromConfig.yawPower);

           	cliQuery = 'x';
           	validCliCommand = false;
            break;

        ///////////////////////////////

        case 'k': // Gimbal Rate Limit
           	cliPrintF("Gimbal Rate Limit: %7.3f\n", eepromConfig.rateLimit * R2D);

           	cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'l': // Gimbal IMU Orientation
           	cliPrintF("Gimbal IMU Orientation: %1d\n", eepromConfig.imuOrientation);

           	cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'm': // Test Phase Value
        	cliPrintF("Test Phase Value: %6.2\n", testPhase * R2D);

        	cliQuery ='x';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'n': // Test Phase Delta
        	cliPrintF("Test Phase Delta: %6.2F\n", testPhaseDelta * R2D);

           	cliQuery ='x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'o': // PID Outputs
            cliPrintF("%12.4f, %12.4f, %12.4f\n", pidCmd[ROLL],
                                                  pidCmd[PITCH],
                                                  pidCmd[YAW]);
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'p': // Counters
            cliPrintF("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d\n", minCnt[ROLL], minCnt[PITCH], minCnt[YAW],
                                                                                              maxCnt[ROLL], maxCnt[PITCH], maxCnt[YAW],
                                                                                              irqCnt[ROLL], irqCnt[PITCH], irqCnt[YAW]);
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 's': // Raw Receiver Commands
        	cliPrintF("%4i, ", rxRead(ROLL));
        	cliPrintF("%4i, ", rxRead(PITCH));
        	cliPrintF("%4i\n", rxRead(YAW));

        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 't': // Pointing Commands
        	cliPrintF("%8.2f, ", pointingCmd[ROLL]  * R2D);
        	cliPrintF("%8.2f, ", pointingCmd[PITCH] * R2D);
        	cliPrintF("%8.2f\n", pointingCmd[YAW]   * R2D);

        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'x':
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////

        case 'A': // Read Roll PID Values
            readCliPID(ROLL_PID);
            cliPrint("\nRoll Rate PID Received....\n");

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'B': // Read Pitch PID Values
            readCliPID(PITCH_PID);
            cliPrint("\nPitch Rate PID Received....\n");

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'C': // Read Yaw PID Values
            readCliPID(YAW_PID);
            cliPrint("\nYaw Rate PID Received....\n");

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'D': // Read Roll RC Parameters
            eepromConfig.rollRateCmdInput     = (uint8_t)readFloatCLI();
            eepromConfig.gimbalRollRate       = readFloatCLI() * D2R;
            eepromConfig.gimbalRollLeftLimit  = readFloatCLI() * D2R;
		    eepromConfig.gimbalRollRightLimit = readFloatCLI() * D2R;

            cliPrint("\nRoll RC Parameters Received....\n");

        	cliQuery = 'd';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'E': // Read Pitch RC Parameters
            eepromConfig.pitchRateCmdInput     = (uint8_t)readFloatCLI();
            eepromConfig.gimbalPitchRate       = readFloatCLI() * D2R;
            eepromConfig.gimbalPitchDownLimit  = readFloatCLI() * D2R;
		    eepromConfig.gimbalPitchUpLimit    = readFloatCLI() * D2R;

            cliPrint("\nPitch RC Parameters Received....\n");

        	cliQuery = 'd';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'F': // Read Yaw RC Parameters
            eepromConfig.yawRateCmdInput     = (uint8_t)readFloatCLI();
            eepromConfig.gimbalYawRate       = readFloatCLI() * D2R;
            eepromConfig.gimbalYawLeftLimit  = readFloatCLI() * D2R;
		    eepromConfig.gimbalYawRightLimit = readFloatCLI() * D2R;

            cliPrint("\nYaw RC Parameters Received....\n");

        	cliQuery = 'd';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'I': // Read Gimbal Axis Enable Flags
    		eepromConfig.rollEnabled  = (uint8_t)readFloatCLI();
    		eepromConfig.pitchEnabled = (uint8_t)readFloatCLI();
            eepromConfig.yawEnabled   = (uint8_t)readFloatCLI();
            eepromConfig.yawAutoPan   = (uint8_t)readFloatCLI();

            cliPrint("\nGimbal Axis Enable Flags Received....\n");

            pwmMotorDriverInit();

          	cliQuery = 'i';
          	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'J': // Read Gimbal Axis Power Levels
        	eepromConfig.rollPower  = readFloatCLI();
        	eepromConfig.pitchPower = readFloatCLI();
        	eepromConfig.yawPower   = readFloatCLI();

        	cliPrint("\nGimbal Axis Power Levels Received....\n");

          	cliQuery = 'j';
          	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'K': // Read Gimbal Rate Limit
        	eepromConfig.rateLimit = readFloatCLI() * D2R;

           	cliPrint("\nGimbal Rate Limit Received....\n");

           	cliQuery = 'k';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'L': // Read Gimbal IMU Orientation
        	eepromConfig.imuOrientation = (uint8_t)readFloatCLI();

           	cliPrint("\nGimbal IMU Orientation Received....\n");

           	orientIMU();

           	cliQuery = 'l';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'M': // Read Test Phase
        	testPhase = readFloatCLI() * D2R;

           	cliPrint("\nTest Phase Received....\n");

           	cliQuery = 'm';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'N': // Read Test Phase Delta
        	testPhaseDelta = readFloatCLI() * D2R;

           	cliPrint("\nTest Phase Delta Received....\n");

           	cliQuery = 'n';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'P': // Sensor CLI
           	sensorCLI();

           	cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'R': // Reset to Bootloader
        	cliPrint("Entering Bootloader....\n\n");
        	delay(1000);
        	bootloader();
        	break;

        ///////////////////////////////

        case 'S': // Reset System
        	cliPrint("\nSystem Rebooting....\n\n");
        	delay(1000);
        	reboot();
        	break;

        ///////////////////////////////

        case 'T': // Not Used
            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'V': // Reset EEPROM Parameters
            cliPrint("\nEEPROM Parameters Reset....\n");
            checkFirstTime(true);
            cliPrint("\nSystem Rebooting....\n\n");
            delay(1000);
            reboot();
            break;

        ///////////////////////////////

        case 'W': // Write EEPROM Parameters
            cliPrint("\nWriting EEPROM Parameters....\n");
            writeEEPROM();

            cliQuery = 'x';
         	validCliCommand = false;
         	break;

        ///////////////////////////////

        case 'X': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'Y': // Not Used
            cliQuery = 'x';
            break;

        ///////////////////////////////

        case 'Z': // Not Used
            cliQuery = 'x';
            break;

       ///////////////////////////////

        case '+': // Increment Test Phase
            testPhase += testPhaseDelta;

            cliQuery = 'm';
            break;

        ///////////////////////////////

        case '-': // Decrement Test Phase
            testPhase -= testPhaseDelta;

            cliQuery = 'm';
            break;

        ///////////////////////////////

        case '?': // Command Summary
        	cliBusy = true;

        	cliPrint("\n");
   		    cliPrint("'a' Rate PIDs                      'A' Set Roll Rate PID Data       AB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'b' Loop Delta Times               'B' Set Pitch Rate PID Data      BB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'c' Loop Execution Times           'C' Set Yaw Rate PID Data        CB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'d' RC Parameters                  'D' Set Roll RC Parameters       DResponse;Rate;Left Limit;Right Limit\n");
   		    cliPrint("'e' 500 Hz Accels                  'E' Set Pitch RC Parameters      EResponse;Rate;Down Limit;Up Limit\n");;
   		    cliPrint("'f' 500 Hz Gyros                   'F' Set Yaw RC Parameters        FResponse;Rate;Left Limit;Right Limit\n");
   		    cliPrint("'g' 10 hz Mag Data                 'G' Not Used\n");
   		    cliPrint("'h' Attitudes                      'H' Not Used\n");
   		    cliPrint("'i' Gimbal Axis Enable Flags       'I' Set Gimbal Axis Enable Flags IR;P;Y;YAP\n");
   		    cliPrint("'j' Gimbal Axis Power Settings     'J' Set Gimbal Axis Power Levels HR;P;Y\n");
   		    cliPrint("'k' Gimbal Rate Limit              'K' Set Gimbal Rate Limit\n");
   		    cliPrint("'l' Gimbal IMU Orientation         'L' Set Gimbal IMU Orientation   LX, X = 1 thru 4\n");
   		    cliPrint("\n");

   		    cliPrint("Press space bar for more, or enter a command....\n");
   		    while (cliAvailable() == false);
   		    cliQuery = cliRead();
   		    if (cliQuery != ' ')
   		    {
   		        validCliCommand = true;
   		        cliBusy = false;
   		    	return;
   		    }

   		    cliPrint("\n");
   		    cliPrint("'m' Test Phase                     'M' Set Test Phase\n");
   		    cliPrint("'n' Test Phase Delta               'N' Set Test Phase Delta\n");
   		    cliPrint("'o' PID Outputs                    'O' Not Used\n");
   		    cliPrint("'p' Counters                       'P' Sensor CLI\n");
   		    cliPrint("'q' Not Used                       'Q' Not Used\n");
   		    cliPrint("'r' Not Used                       'R' Reset and Enter Bootloader\n");
   		    cliPrint("'s' Raw Receiver Commands          'S' Reset\n");
   		    cliPrint("'t' Pointing Commands              'T' Not Used\n");
   		    cliPrint("'u' Not Used                       'U' Not Used\n");
   		    cliPrint("'v' Not Used                       'V' Reset EEPROM Parameters\n");
   		    cliPrint("'w' Not Used                       'W' Write EEPROM Parameters\n");
   		    cliPrint("'x' Terminate CLI Communication    'X' Not Used\n");
   		    cliPrint("\n");

   		    cliPrint("Press space bar for more, or enter a command....\n");
   		    while (cliAvailable() == false);
   		    cliQuery = cliRead();
   		    if (cliQuery != ' ')
   		    {
   		    	validCliCommand = true;
   		    	cliBusy = false;
   		    	return;
   		    }

   		    cliPrint("\n");
   		    cliPrint("'y' Not Used                       'Y' Not Used\n");
   		    cliPrint("'z' Not Used                       'Z' Not Used\n");
   		    cliPrint("'+' Increment Test Phase           '?' Command Summary\n");
   		    cliPrint("'-' Decrement Test Phase\n");
   		    cliPrint("\n");

  		    cliQuery = 'x';
  		    cliBusy = false;
   		    break;

            ///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
