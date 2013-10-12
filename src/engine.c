/*
 *  engine.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

float pitch_setpoint = 0.0f, pitch_Error_last, pitch_angle_correction;
float roll_setpoint = 0.0f,  roll_Error_last,  roll_angle_correction;
float yaw_setpoint = 0.0f,   yaw_Error_last,   yaw_angle_correction;

float ADC1Ch13_yaw;

static float rollRCOffset = 0.0f, pitchRCOffset = 0.0f, yawRCOffset = 0.0f;

float cameraOrient[3];
float accAngleSmooth[3];

float Step[NUMAXIS]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[NUMAXIS] = {0.0f, 0.0f, 0.0f};

///////////////////////////////////////////////////////////////////////////////

void roll_PID(void)
{
    float Error_current = roll_setpoint + cameraOrient[ROLL] * 1000.0f;
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
    float Error_current = pitch_setpoint + cameraOrient[PITCH] * 1000.0f;
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
    float Error_current = yaw_setpoint + cameraOrient[YAW] * 1000.0f;
    // HJI float KP = Error_current * (float)configData[2] / 1000.0;
    float KP = Error_current * eepromConfig.PID[YAW_PID].P / 1000.0f;
    // HJI float KD = (float)configData[5] / 100.0 * (Error_current - yaw_Error_last);
    float KD = eepromConfig.PID[YAW_PID].D / 100.0f * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    // HJI SetYawMotor(KP + KD, configData[8]);
    SetYawMotor(KP + KD, (int)eepromConfig.yawPower);
}

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

void initOrientation()
{
    int initLoops = 150;
    float accAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };
    int i;

    for (i = 0; i < initLoops; i++)
    {
        readMPU6050();

        computeMPU6050TCBias();

        sensors.accel500Hz[XAXIS] =  ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[YAXIS] =  ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[ZAXIS] = -((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

        accAngle[ROLL]  += atan2f(-sensors.accel500Hz[YAXIS], -sensors.accel500Hz[ZAXIS]);
        accAngle[PITCH] += atan2f( sensors.accel500Hz[XAXIS], -sensors.accel500Hz[ZAXIS]);

        accAngleSmooth[ROLL ] = accAngle[ROLL ] / (float)initLoops;
        accAngleSmooth[PITCH] = accAngle[PITCH] / (float)initLoops;

        delay(2);
    }

    cameraOrient[PITCH] = accAngleSmooth[PITCH];
    cameraOrient[ROLL ] = accAngleSmooth[ROLL ];
    cameraOrient[YAW  ] = 0.0f;

    cliPrintF("Init Angles: roll %7.2f, pitch %7.2f, yaw %7.2f\n", cameraOrient[ROLL ] * R2D,
                                                                   cameraOrient[PITCH] * R2D,
                                                                   cameraOrient[YAW  ] * R2D);
}

///////////////////////////////////////////////////////////////////////////////

void Get_Orientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
    float accAngle[3];
    float gyroRate[3];

    accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
    accAngle[PITCH] = atan2f( accData[XAXIS], -accData[ZAXIS]);

    smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
    smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;

    gyroRate[PITCH] =  gyroData[PITCH];
    orient[PITCH]   = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);

    gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);
    orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);

    gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);
    orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);
}

///////////////////////////////////////////////////////////////////////////////

void engineProcess(float dt)
{
    Get_Orientation(accAngleSmooth, cameraOrient, sensors.accel500Hz, sensors.gyro500Hz, dt);

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

    pitch_angle_correction = constrain((cameraOrient[PITCH] + pitchRCOffset) * 50.0f, -CORRECTION_STEP, CORRECTION_STEP);
    pitch_setpoint += pitch_angle_correction; // Pitch return to zero after collision

    // Roll Adjustments
    //roll_setpoint += Step[ROLL];
    rollRCOffset  += Step[ROLL] / 1000.0f;

    // include the config roll offset which is scaled to 0 = -10.0 degrees, 100 = 0.0 degrees, and 200 = 10.0 degrees
    // HJI roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset + ((configData[11] - 100) / 10.0) / R2D) * 50.0, -CORRECTION_STEP, CORRECTION_STEP);
    roll_angle_correction = constrain((cameraOrient[ROLL] + rollRCOffset) * 50.0f, -CORRECTION_STEP, CORRECTION_STEP);
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
}

