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
	uint8_t temp1, temp2, temp3;

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
        	// HJI cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ] * R2D,
        	// HJI 		                           sensors.attitude500Hz[PITCH] * R2D,
        	// HJI 		                           sensors.attitude500Hz[YAW  ] * R2D);

            cliPrintF("%7.2f, %7.2f, %7.2f\n", cameraOrient[ROLL ] * R2D,
			                                   cameraOrient[PITCH] * R2D,
                                               cameraOrient[YAW  ] * R2D);

            validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'i': // Gimbal Axis Enable Flags
           	cliPrintF("Gimbal Roll Axis:  %s\n", eepromConfig.rollEnabled  ? "Enabled" : "Disabled");
           	cliPrintF("Gimbal Pitch Axis: %s\n", eepromConfig.pitchEnabled ? "Enabled" : "Disabled");
           	cliPrintF("Gimbal Yaw Axis:   %s\n", eepromConfig.yawEnabled   ? "Enabled" : "Disabled");

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
           	cliPrintF("Gimbal Rate Limit: %7.3f\n", eepromConfig.rateLimit);

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

        case 'o': // Debug Orientation
            cliPrintF("Pitch_setpoint:%12.4f | Roll_setpoint:%12.4f | Yaw_setpoint:%12.4f\n", pitch_setpoint * D2R / 1000.0f,
                                                                                              roll_setpoint  * D2R / 1000.0f,
                                                                                              yaw_setpoint   * D2R / 1000.0f);
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'p': // Debug Counter
            cliPrintF("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d\n", MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW],
                                                                                              MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW],
                                                                                              IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW]);
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
        	cliPrintF("%8.2f, ", rxCommand[ROLL]);
        	cliPrintF("%8.2f, ", rxCommand[PITCH]);
        	cliPrintF("%8.2f\n", rxCommand[YAW]);

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
            cliPrint( "\nRoll Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'B': // Read Pitch PID Values
            readCliPID(PITCH_PID);
            cliPrint( "\nPitch Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'C': // Read Yaw PID Values
            readCliPID(YAW_PID);
            cliPrint( "\nYaw Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'I': // Read Gimbal Axis Enable Flags
    		temp1 = (uint8_t)readFloatCLI();
    		temp2 = (uint8_t)readFloatCLI();
            temp3 = (uint8_t)readFloatCLI();

            if (temp1 == 0)
            	eepromConfig.rollEnabled = false;
            else
            	eepromConfig.rollEnabled = true;

            if (temp2 == 0)
            	eepromConfig.pitchEnabled = false;
            else
            	eepromConfig.pitchEnabled = true;

            if (temp3 == 0)
            	eepromConfig.yawEnabled = false;
            else
            	eepromConfig.yawEnabled = true;

            cliPrint( "\nGimbal Axis Enable Flags Received....\n" );

            // HJI pwmMotorDriverInit();

          	cliQuery = 'i';
          	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'J': // Read Gimbal Axis Power Levels
        	eepromConfig.rollPower  = readFloatCLI();
        	eepromConfig.pitchPower = readFloatCLI();
        	eepromConfig.yawPower   = readFloatCLI();

        	cliPrint( "\nGimbal Axis Power Levels Received....\n" );

          	cliQuery = 'j';
          	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'K': // Read Gimbal Rate Limit
        	eepromConfig.rateLimit = readFloatCLI();

           	cliPrint( "\nGimbal Rate Limit Received....\n" );

           	cliQuery = 'k';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'L': // Read Gimbal IMU Orientation
        	eepromConfig.imuOrientation = (uint8_t)readFloatCLI();

           	cliPrint( "\nGimbal IMU Orientation Received....\n" );

           	orientIMU();

           	cliQuery = 'l';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'M': // Read Test Phase
        	testPhase = readFloatCLI() * D2R;

           	cliPrint( "\nTest Phase Received....\n" );

           	cliQuery = 'm';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'N': // Read Test Phase Delta
        	testPhaseDelta = readFloatCLI() * D2R;

           	cliPrint( "\nTest Phase Delta Received....\n" );

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
            cliPrint( "\nEEPROM Parameters Reset....\n" );
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
   		    cliPrint("'d' Not Used                       'D' Not Used\n");
   		    cliPrint("'e' 500 Hz Accels                  'E' Not Used\n");
   		    cliPrint("'f' 500 Hz Gyros                   'F' Not Used\n");
   		    cliPrint("'g' 10 hz Mag Data                 'G' Not used\n");
   		    cliPrint("'h' Attitudes                      'H' Not Used\n");
   		    cliPrint("'i' Gimbal Axis Enable Flags       'I' Set Gimbal Axis Enable Flags IR;P;Y\n");
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
   		    cliPrint("'o' Debug Orientation              'O' Not Used\n");
   		    cliPrint("'p' Debug Counter                  'P' Sensor CLI\n");
   		    cliPrint("'q' Not Used                       'Q' Not Used\n");
   		    cliPrint("'r' Not Used                       'R' Reset and Enter Bootloader\n");
   		    cliPrint("'s' Receiver Commands              'S' Reset\n");
   		    cliPrint("'t' Not Used                       'T' Not Used\n");
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
