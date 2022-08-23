/*
 * AT30TS74.h
 *
 *  Created on: Aug 8, 2021
 *      Author: damonb
 */

#ifndef INC_AT30TS74_H_
#define INC_AT30TS74_H_


#include "defs.h"

struct sAT30TS74
{
  uint8_t Address;
  I2C_HandleTypeDef* Interface;
  uint16_t Data[64];
  double Temperature;
  bool Configured;
  uint8_t Index;
  uint8_t Samples;
};

void AT30TS74_InitStruct(struct sAT30TS74* s, I2C_HandleTypeDef* interface, uint8_t add);
void AT30TS74_Configure(struct sAT30TS74* s);
void AT30TS74_GetTemperature(struct sAT30TS74* s);

#endif /* INC_AT30TS74_H_ */
