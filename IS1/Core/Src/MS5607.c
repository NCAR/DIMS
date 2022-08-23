/*
 * MS5607.c
 *
 *  Created on: Apr 20, 2021
 *      Author: damonb
 */

/*
 * The MS5607-02BA has only five basic commands:
    1. Reset
    2. Read PROM (128 bit of calibration words)
    3. D1 conversion
    4. D2 conversion
    5. Read ADC result (24 bit pressure / temperature)
*/
#define MS5607_ADDR 0b1110111

#include "MS5607.h"
#include <math.h>

void MS5607_StartConversion(I2C_HandleTypeDef* i2c, bool temp)
{
  uint8_t outbuffer[1] = {0};
  outbuffer[0] = 0x40 | (temp << 4);
  HAL_I2C_Master_Transmit(i2c, MS5607_ADDR << 1, outbuffer, 1, 100);
}

uint32_t MS5607_ReadADC(I2C_HandleTypeDef* i2c)
{
  uint8_t outbuffer[1] = {0};
  uint8_t inbuffer[4] = {0};
  outbuffer[0] = 0;
  HAL_I2C_Master_Transmit(i2c, MS5607_ADDR << 1, outbuffer, 1, 100);
  HAL_I2C_Master_Receive(i2c,  MS5607_ADDR << 1, inbuffer, 3, 100);
  return ((inbuffer[0] << 16) | (inbuffer[1] << 8) | inbuffer[2]);
}

uint16_t MS5607_ReadCoeff(I2C_HandleTypeDef* i2c, uint8_t coeff_number)
{
  uint8_t outbuffer[1] = {0};
  uint8_t inbuffer[2] = {0};
  outbuffer[0] = 0xa0 | (coeff_number << 1); // read coefficient
  HAL_I2C_Master_Transmit(i2c, MS5607_ADDR << 1, outbuffer, 1, 100);
  HAL_I2C_Master_Receive(i2c,  MS5607_ADDR << 1, inbuffer, 2, 100);
  return ((inbuffer[0] << 8) | inbuffer[1]);
}

void MS5607_GetCoefficients(I2C_HandleTypeDef* i2c, struct sMS5607* ms)
{
  int i;
  for (i=1;i<7;i++)
    ms->C[i] = MS5607_ReadCoeff(i2c, i);
}

void MS5607_CalculatePressure(struct sMS5607* ms, int32_t* temp, int32_t* pressure)
{
  double dT, TEMP, OFF, SENS, P, T2, SENS2, OFF2;
  T2 = 0;
  SENS2 = 0;
  OFF2 = 0;
  dT = ms->D[2] - ms->C[5] * pow(2,8);
  TEMP = 2000 + dT * ms->C[6]/pow(2,23);

  if (TEMP < 2000)
  {
    T2 = pow(dT,2)/pow(2,31);
    OFF2 = 61 * pow(TEMP - 2000,2)/pow(2,4);
    SENS2 = 2 * pow(TEMP-2000,2);
  }
  if (TEMP < -1500)
  {
    OFF2 = OFF2 + 15 * pow(TEMP + 1500,2);
    SENS2 = SENS2 + 8 * pow(TEMP + 1500,2);
  }

  TEMP -= T2;
  OFF = ms->C[2]*pow(2,17)+(ms->C[4]*dT)/pow(2,6);
  OFF -= OFF2;
  SENS = ms->C[1]*pow(2,16)+(ms->C[3]*dT)/pow(2,7);
  SENS -= SENS2;
  P = (ms->D[1]*SENS/pow(2,21) - OFF) / pow(2,15);
  *temp = TEMP;
  *pressure = P;
}

void InitMS5607(struct sMS5607* ms)
{
  ms->D[0]=0;
  ms->D[1]=0;
  ms->D[2]=0;
  ms->C[0]=0;
  ms->C[1]=0;
  ms->C[2]=0;
  ms->C[3]=0;
  ms->C[4]=0;
  ms->C[5]=0;
  ms->C[6]=0;
}
