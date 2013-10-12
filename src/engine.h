/*
 *  engine.h
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define PITCH_UP_LIMIT   (-50.0f * D2R)
#define PITCH_DOWN_LIMIT ( 90.0f * D2R)
#define CORRECTION_STEP     1.0f

extern float cameraOrient[3];  // HJI
extern float pitch_setpoint;   // HJI
extern float roll_setpoint;    // HJI
extern float yaw_setpoint;     // HJI

void initOrientation(void);
void engineProcess(float dt);

///////////////////////////////////////////////////////////////////////////////




