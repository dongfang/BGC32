/*
 *  engine.h
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */

#ifndef ENGiNE_H_
#define ENGINE_H_

#define PITCH_UP_LIMIT (-50 * D2R)
#define PITCH_DOWN_LIMIT (90 * D2R)
#define CORRECTION_STEP 1.0F

// HJI vextern int debugPrint;
// HJI extern int debugPerf;
// HJI extern int debugSense;
// HJI extern int debugCnt;
// HJI extern int debugRC;
// HJI extern int debugOrient;
// HJI extern int debugCam;

extern float CameraOrient[3];  // HJI
extern float pitch_setpoint;   // HJI
extern float roll_setpoint;    // HJI
extern float yaw_setpoint;     // HJI

void Init_Orientation(void);
void engineProcess(float dt);
void Get_Orientation(float *AccAngleSmooth, float *Orient, float *AccData, float *GyroData, float dt);
#endif /* ENGINE_H_ */




