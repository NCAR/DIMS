/*
 * PID.h
 *
 *  Created on: Aug 10, 2021
 *      Author: damonb
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define POSITIONHISTORY (60)
#include "defs.h"

struct sPID
{
  float TargetP;
  float Kp, Ki, Kd, MaxI;
  float LastP[POSITIONHISTORY];
  float DeltaT;
  uint8_t IntegratorCount;
};


void PID_InitStruct(struct sPID* s);
void PID_SavePoint(struct sPID* s, float p);
float PID_SumError(struct sPID* s);
float PID_Velocity(struct sPID* s);
float PID_Effort(struct sPID* s, float p);

#endif /* INC_PID_H_ */

