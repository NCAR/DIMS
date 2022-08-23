/*
 * MS5607.h
 *
 *  Created on: Apr 20, 2021
 *      Author: damonb
 */

#ifndef INC_MS5607_H_
#define INC_MS5607_H_

#include "defs.h"

struct sMS5607
{
  uint16_t C[7];
  uint32_t D[3];
};

void MS5607_StartConversion(I2C_HandleTypeDef* i2c, bool temp);
uint32_t MS5607_ReadADC(I2C_HandleTypeDef* i2c);
uint16_t MS5607_ReadCoeff(I2C_HandleTypeDef* i2c, uint8_t coeff_number);

void MS5607_GetCoefficients(I2C_HandleTypeDef* i2c, struct sMS5607* ms);
void MS5607_CalculatePressure(struct sMS5607* ms, int32_t* temp, int32_t* pressure);

void InitMS5607(struct sMS5607* ms);



#endif /* INC_MS5607_H_ */
