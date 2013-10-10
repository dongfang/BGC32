/*
 *  engine.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
// HJI #include <stdint.h>
// HJI #include <math.h>
// HJI #include "engine.h"
// HJI #include "adc.h"
// HJI #include "gyro.h"
// HJI #include "utils.h"
// HJI #include "config.h"
// HJI #include "pwm.h"
// HJI #include "rc.h"
// HJI #include "comio.h"
// HJI #include "systick.h"
// HJI #include "stopwatch.h"
// HJI #include "I2C.h"
// HJI #include "definitions.h"
// HJI #include "usb.h"
// HJI #include "main.h"

///////////////////////////////////////////////////////////////////////////////

#include "board.h"  // HJI

///////////////////////////////////////////////////////////////////////////////

// HJI int   debugPrint = 0;
// HJI int   debugPerf = 0;
// HJI int   debugSense = 0;
// HJI int   debugCnt = 0;
// HJI int   debugRC = 0;
// HJI int   debugOrient = 0;

float /*pitch, Gyro_Pitch_angle,*/ pitch_setpoint = 0.0f, pitch_Error_last,  pitch_angle_correction;
float /*roll,  Gyro_Roll_angle,*/  roll_setpoint = 0.0f,  roll_Error_last,    roll_angle_correction;
float /*yaw,   Gyro_Yaw_angle,*/   yaw_setpoint = 0.0f,   yaw_Error_last,      yaw_angle_correction;

float ADC1Ch13_yaw;

static float rollRCOffset = 0.0f, pitchRCOffset = 0.0f, yawRCOffset = 0.0f;

// HJI static int printcounter;

float CameraOrient[3];    // HJI float CameraOrient[EULAR];
float AccAngleSmooth[3];  // HJI float AccAngleSmooth[EULAR];

float AccData[NUMAXIS]  = {0.0f, 0.0f, 0.0f};
float GyroData[NUMAXIS] = {0.0f, 0.0f, 0.0f};

float Step[NUMAXIS]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[NUMAXIS] = {0.0f, 0.0f, 0.0f};

///////////////////////////////////////////////////////////////////////////////

void roll_PID(void)
{
    float Error_current = roll_setpoint + CameraOrient[ROLL] * 1000.0f;
    // HJI float KP = Error_current * (float)configData[1] / 1000.0;
    float KP = Error_current * eepromConfig.PID[ROLL_PID].P / 1000.0f;
    // HJI float KD = (float)configData[4] / 100.0 * (Error_current - roll_Error_last);
    float KD = eepromConfig.PID[ROLL_PID].D / 100.0f * (Error_current - roll_Error_last);

    roll_Error_last = Error_current;

    // HJI SetRollMotor(KP + KD, configData[7]);
    SetRollMotor(KP + KD, (int)eepromConfig.rollPower);
}

///////////////////////////////////////////////////////////////////////////////

void pitch_PID(void)
{
    float Error_current = pitch_setpoint + CameraOrient[PITCH] * 1000.0f;
    // HJI float KP = Error_current * (float)configData[0] / 1000.0;
    float KP = Error_current * eepromConfig.PID[PITCH_PID].P / 1000.0f;
    // HJI float KD = (float)configData[3] / 100.0 * (Error_current - pitch_Error_last);
    float KD = eepromConfig.PID[PITCH_PID].D / 100.0f * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;

    // HJI SetPitchMotor(KP + KD, configData[6]);
    SetPitchMotor(KP + KD, (int)eepromConfig.pitchPower);
}

///////////////////////////////////////////////////////////////////////////////

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + CameraOrient[YAW] * 1000.0f;
    // HJI float KP = Error_current * (float)configData[2] / 1000.0;
    float KP = Error_current * eepromConfig.PID[YAW_PID].P / 1000.0f;
    // HJI float KD = (float)configData[5] / 100.0 * (Error_current - yaw_Error_last);
    float KD = eepromConfig.PID[YAW_PID].D / 100.0f * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    // HJI SetYawMotor(KP + KD, configData[8]);
    SetYawMotor(KP + KD, (int)eepromConfig.yawPower);
}

///////////////////////////////////////////////////////////////////////////////

// HJI float constrain(float value, float low, float high)
// HJI {
// HJI     if (value < low)
// HJI         return low;

// HJI     if (value > high)
// HJI         return high;

// HJI     return value;
// HJI }

///////////////////////////////////////////////////////////////////////////////
/*
  Limits the Pitch angle
*/
float Limit_Pitch(float step, float pitch)
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

void Init_Orientation()
{
    int init_loops = 150;
    // HJI float AccAngle[NUMAXIS];
    float AccAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };  // HJI
    int i;

    for (i = 0; i < init_loops; i++)
    {
        // HJI MPU6050_ACC_get(AccData); //Getting Accelerometer data

      readMPU6050();  // HJI

      computeMPU6050TCBias();  // HJI

      sensors.accel500Hz[XAXIS] = -((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;  // HJI
      sensors.accel500Hz[YAXIS] = -((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;  // HJI
      sensors.accel500Hz[ZAXIS] =  ((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;  // HJI

        // HJI AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle
      AccAngle[ROLL]  += -(atan2f(sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS]));   // Calculating roll ACC angle HJI
        // HJI AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle
      AccAngle[PITCH] += +(atan2f(sensors.accel500Hz[XAXIS], sensors.accel500Hz[ZAXIS]));   // Calculating pitch ACC angle HJI

        // HJI AccAngleSmooth[ROLL]  = ((AccAngleSmooth[ROLL] * (float)(init_loops - 1))  + AccAngle[ROLL])  / (float)init_loops; //Averaging pitch ACC values
      AccAngleSmooth[ROLL ] = AccAngle[ROLL ] / (float)init_loops; //Averaging pitch ACC values HJI
        // HJI AccAngleSmooth[PITCH] = ((AccAngleSmooth[PITCH] * (float)(init_loops - 1)) + AccAngle[PITCH]) / (float)init_loops; //Averaging roll  ACC values
      AccAngleSmooth[PITCH] = AccAngle[PITCH] / (float)init_loops; //Averaging roll  ACC values HJI
        // HJI Delay_ms(1);
        delay(2);  // HJI
    }

    CameraOrient[PITCH] = AccAngleSmooth[PITCH];
    CameraOrient[ROLL ] = AccAngleSmooth[ROLL ];
    CameraOrient[YAW  ] = 0.0f;

    cliPrintF("Init Angles: roll %7.2f, pitch %7.2f, yaw %7.2f\n", CameraOrient[ROLL ] * R2D,
                                                                   CameraOrient[PITCH] * R2D,
                                                                   CameraOrient[YAW  ] * R2D);
}

///////////////////////////////////////////////////////////////////////////////

void Get_Orientation(float *SmoothAcc, float *Orient, float *AccData, float *GyroData, float dt)
{
  float AccAngle[3];  // HJI float AccAngle[EULAR];
  float GyroRate[3];  // HJI float GyroRate[EULAR];

    // HJI AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle
    AccAngle[ROLL ] = -(atan2f(AccData[YAXIS], AccData[ZAXIS]));   //Calculating roll ACC angle  HJI
    // HJI AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle
    AccAngle[PITCH] = +(atan2f(AccData[XAXIS], AccData[ZAXIS]));   //Calculating pitch ACC angle HJI

    SmoothAcc[ROLL]  = ((SmoothAcc[ROLL ] * 99.0f) + AccAngle[ROLL])  / 100.0f; //Averaging roll  ACC values
    SmoothAcc[PITCH] = ((SmoothAcc[PITCH] * 99.0f) + AccAngle[PITCH]) / 100.0f; //Averaging pitch ACC values

    // HJI GyroRate[PITCH] =  GyroData[X_AXIS];
    GyroRate[PITCH] =  GyroData[YAXIS];  // HJI
    Orient[PITCH]   = (Orient[PITCH] + GyroRate[PITCH] * dt) + 0.0002f * (SmoothAcc[PITCH] - Orient[PITCH]);  //Pitch Horizon

    // HJI GyroRate[ROLL] = -GyroData[Z_AXIS] * sinf(Orient[PITCH]) + GyroData[Y_AXIS] * cosf(fabsf(Orient[PITCH]));
    GyroRate[ROLL] = -GyroData[YAW] * sinf(Orient[PITCH]) + GyroData[XAXIS] * cosf(fabsf(Orient[PITCH]));  // HJI
    Orient[ROLL] = (Orient[ROLL] + GyroRate[ROLL] * dt)    + 0.0002f * (SmoothAcc[ROLL] - Orient[ROLL]); //Roll Horizon

    // HJI GyroRate[YAW]  = -GyroData[Z_AXIS] * cosf(fabsf(Orient[PITCH])) - GyroData[Y_AXIS] * sinf(Orient[PITCH]); //presuming Roll is horizontal
    GyroRate[YAW]  = -GyroData[YAW] * cosf(fabsf(Orient[PITCH])) - GyroData[XAXIS] * sinf(Orient[PITCH]); //presuming Roll is horizontal  HJI
    Orient[YAW] = (Orient[YAW] + GyroRate[YAW] * dt); //Yaw
}

///////////////////////////////////////////////////////////////////////////////

void engineProcess(float dt)
{
    // HJI static int loopCounter;
  // HJI tStopWatch sw;

  // HJI loopCounter++;
  // HJI LEDon();
  // HJI DEBUG_LEDoff();

  // HJI StopWatchInit(&sw);
    // HJI MPU6050_ACC_get(AccData); // Getting Accelerometer data
    // HJI unsigned long tAccGet = StopWatchLap(&sw);

    // HJI MPU6050_Gyro_get(GyroData); // Getting Gyroscope data
    // HJI unsigned long tGyroGet = StopWatchLap(&sw);

    // HJI Get_Orientation(AccAngleSmooth, CameraOrient, AccData, GyroData, dt);
    Get_Orientation(AccAngleSmooth, CameraOrient, sensors.accel500Hz, sensors.gyro500Hz, dt);  // HJI
    // HJI unsigned long tAccAngle = StopWatchLap(&sw);

    // if we enable RC control
    // HJI if (configData[9] == '1')
    // HJI {
    // HJI     // Get the RX values and Averages
    // HJI     Get_RC_Step(Step, RCSmooth); // Get RC movement on all three AXIS
    // HJI     Step[PITCH] = Limit_Pitch(Step[PITCH], CameraOrient[PITCH]); // limit pitch to defined limits in header
    // HJI }

    // Pitch adjustments
    //pitch_setpoint += Step[PITCH];
    pitchRCOffset  += Step[PITCH] / 1000.0f;

    pitch_angle_correction = constrain((CameraOrient[PITCH] + pitchRCOffset) * 50.0f, -CORRECTION_STEP, CORRECTION_STEP);
    pitch_setpoint += pitch_angle_correction; // Pitch return to zero after collision

    // Roll Adjustments
    //roll_setpoint += Step[ROLL];
    rollRCOffset  += Step[ROLL] / 1000.0f;

    // include the config roll offset which is scaled to 0 = -10.0 degrees, 100 = 0.0 degrees, and 200 = 10.0 degrees
    // HJI roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset + ((configData[11] - 100) / 10.0) / R2D) * 50.0, -CORRECTION_STEP, CORRECTION_STEP);
    roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset) * 50.0f, -CORRECTION_STEP, CORRECTION_STEP);
    roll_setpoint += roll_angle_correction; //Roll return to zero after collision

    // if we enabled AutoPan on Yaw
    // HJI if (configData[10] == '0')
    // HJI {
    // HJI     ADC1Ch13_yaw = ((ADC1Ch13_yaw * 99.0) + ((float)(readADC1(13) - 2000) / 4000.0)) / 100.0;  // Average ADC value
    // HJI }

    // Yaw Adjustments
    yaw_setpoint += Step[YAW];
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

    // HJI unsigned long tCalc = StopWatchLap(&sw);

    pitch_PID();
    roll_PID();
    yaw_PID();

    /* HJI unsigned long tPID = StopWatchLap(&sw);
    unsigned long tAll = StopWatchTotal(&sw);

    printcounter++;

    //if (printcounter >= 500 || dt > 0.0021)
    if (printcounter >= 500)
    {
        if (debugPrint)
        {
            print("Loop: %7d, I2CErrors: %d, angles: roll %7.2f, pitch %7.2f, yaw %7.2f\r\n",
                  loopCounter, I2Cerrorcount, Rad2Deg(CameraOrient[ROLL]),
                  Rad2Deg(CameraOrient[PITCH]), Rad2Deg(CameraOrient[YAW]));
        }

        if (debugSense)
        {
            print(" dt %f, AccData: %6.0f | %6.0f | %6.0f, GyroData %7.3f | %7.3f | %7.3f \r\n",
                  dt, AccData[X_AXIS], AccData[Y_AXIS], AccData[Z_AXIS], GyroData[X_AXIS], GyroData[Y_AXIS], GyroData[Z_AXIS]);
        }

        if (debugPerf)
        {
            print("idle: %5.2f%%, time[µs]: attitude est. %4d, IMU acc %4d, gyro %4d, angle %4d, calc %4d, PID %4d\r\n",
                  GetIdlePerf(), tAll, tAccGet, tGyroGet, tAccAngle, tCalc, tPID);
        }

        if (debugRC)
        {
            print(" RC4avg: %7.2f |  RC2avg: %7.2f |  RC3avg: %7.2f | PStep:%7.3f  YStep: %7.3f  RStep: %7.3f\r\n",
                  RCSmooth[PITCH], RCSmooth[ROLL], RCSmooth[YAW], Step[PITCH], Step[YAW], Step[ROLL]);
        }

        if (debugOrient)
        {
            print("Pitch_setpoint:%12.4f | Roll_setpoint:%12.4f | Yaw_setpoint:%12.4f\r\n",
                  Rad2Deg(pitch_setpoint) / 1000.0, Rad2Deg(roll_setpoint) / 1000.0, Rad2Deg(yaw_setpoint) / 1000.0);
        }

        if (debugCnt)
        {
            print("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d, usbOverrun %4d\r\n",
                  MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW],
                  MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW],
                  IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW],
                  usbOverrun());
        }

        printcounter = 0;
    }

    LEDoff();  HJI */
}

