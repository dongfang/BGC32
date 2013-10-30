///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

float accAngleSmooth[3];

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

    sensors.attitude500Hz[PITCH] = accAngleSmooth[PITCH];
    sensors.attitude500Hz[ROLL ] = accAngleSmooth[ROLL ];
    sensors.attitude500Hz[YAW  ] = 0.0f;

    cliPrintF("\nInitial Gimbal Angles: Roll %7.2f, Ritch %7.2f, Yaw %7.2f\n\n", sensors.attitude500Hz[ROLL ] * R2D,
                                                                                 sensors.attitude500Hz[PITCH] * R2D,
                                                                                 sensors.attitude500Hz[YAW  ] * R2D);
}

///////////////////////////////////////////////////////////////////////////////

void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
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
