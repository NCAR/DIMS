/*
 * TMP117.h
 *
 *  Created on: Aug 8, 2021
 *      Author: damonb
 */

#ifndef INC_TMP117_H_
#define INC_TMP117_H_

#include "defs.h"

struct sTMP117
{
  uint8_t Address;
  I2C_HandleTypeDef* Interface;
  uint16_t Data[64];
  double Temperature;
  bool Configured;
  uint8_t Index;
  uint8_t Samples;
};


void TMP117_Configure(struct sTMP117* s);
void TMP117_InitStruct(struct sTMP117* s, I2C_HandleTypeDef* interface, uint8_t addpin);
void TMP117_GetTemperature(struct sTMP117* s);

void I2C2_State(bool s);

#endif /* INC_TMP117_H_ */
