/*
 * PID.c
 * PID controller library for caluclating the effort to apply to a system to reach a target
 * 
 *  Created on: Aug 10, 2021
 *      Author: damonb
 *     Edited by mjeffers
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


/*****
 * @brief Initialize the PID structure to all zeros
 * @param s: pointer to the PID structure
*/
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


/*****
 * @brief Save the current position to the history
 * @param s: pointer to the PID structure
 * @param p: the current position
*/
void PID_SavePoint(struct sPID* s, float p)
{
//  memmove((void*) *(s->LastP[1]),(void*) *(s->LastP[0]), sizeof(s->LastP)*(POSITIONHISTORY-1));
  uint8_t i;
  for (i=(POSITIONHISTORY-1); i>0; i--)
    s->LastP[i] = s->LastP[i-1];
  s->LastP[0] = p;
}


/*****
 * @brief Calculate the proportional error
 * @param s: pointer to the PID structure
 * @return the proportional error
*/
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


/******
 * @brief Calculate the velocity of the system
 * @param s: pointer to the PID structure
 * @return the velocity of the system
 * @note this is a very simple calculation that just looks at the last 4 points
 *       and calculates the velocity over the last 4*DeltaT seconds
 *       this is not a very good way to calculate velocity, but it is simple
 *       and works well enough for our purposes
 * 
*/

float PID_Velocity(struct sPID* s)
{
  uint8_t periods = 4;
  return (s->LastP[0]-s->LastP[periods])/(s->DeltaT*periods);
}

/*****
 * @brief Calculate the effort to apply to the system
 * @param s: pointer to the PID structure
 * @param p: the current position
 * @return the effort to apply to the system between 0 and 1
*/
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
