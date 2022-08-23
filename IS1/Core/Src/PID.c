/*
 * PID.c
 *
 *  Created on: Aug 10, 2021
 *      Author: damonb
 */

/*
struct sPID
{
  float kp, ki, kd;
  float ep, ei, ed;
  float LastP[4];
  float DeltaT;
};
*/

#include "pid.h"

void PID_InitStruct(struct sPID* s)
{
  uint8_t i;
  s->Kp=0.0f;
  s->Ki=0.0f;
  s->Kd=0.0f;
  s->MaxI=0.0f;
  s->DeltaT = 0.0f;
  s->TargetP = 0.0f;
  s->IntegratorCount = 0;
  for (i=0;i<POSITIONHISTORY;i++)
    s->LastP[i]=0.0f;
}

void PID_SavePoint(struct sPID* s, float p)
{
//  memmove((void*) *(s->LastP[1]),(void*) *(s->LastP[0]), sizeof(s->LastP)*(POSITIONHISTORY-1));
  uint8_t i;
  for (i=(POSITIONHISTORY-1); i>0; i--)
    s->LastP[i] = s->LastP[i-1];
  s->LastP[0] = p;
}

float PID_SumError(struct sPID* s)
{
  s->IntegratorCount++;
  if (s->IntegratorCount < POSITIONHISTORY)
    return 0;
  else
    s->IntegratorCount = POSITIONHISTORY;
  float err = 0;
  uint8_t i;
  for (i=0; i<POSITIONHISTORY; i++)
    err += (s->TargetP - s->LastP[i]);
  err *= (s->Ki);
  if ((-err) > s->MaxI)
    err = -(s->MaxI);
  if (err > s->MaxI)
    err = s->MaxI;
  return err;
}

// we may want to improve this later
float PID_Velocity(struct sPID* s)
{
  uint8_t periods = 4;
  return (s->LastP[0]-s->LastP[periods])/(s->DeltaT*periods);
}

// calculate the error and return a value between 0 and 1
float PID_Effort(struct sPID* s, float p)
{
  if (s->TargetP == 0)
    return 0;
  if (s->DeltaT == 0)
    return 0;
  float Ep, Ed, Ei, Effort;
  Ep = (s->TargetP - p) * (s->Kp);
  Ed = (0 - PID_Velocity(s)) * (s->Kd);
  Ei = PID_SumError(s);
  Effort = Ep+Ed+Ei;
//  printf("t %7.3f Ep %7.3f  Ed %7.3f  Ei %7.3f  E %6.3f\n", p, Ep, Ed, Ei, Effort);
  if (Effort > 1)
    return 1;
  if (Effort < 0)
    return 0;
  return Effort;
}
