/*
 * MS5607.c
 *  MS5607-02BA03 Pressure Sensor Driver
 *  Created on: Apr 20, 2021
 *      Author: damonb
 *   Edited by mjeffers
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


/******
 * @brief Starts the Connection with the MS5607
 * @param i2c: pointer to the I2C_HandleTypeDef struct
 * @param temp: boolean to select temperature or pressure
*/
void MS5607_StartConversion(I2C_HandleTypeDef* i2c, bool temp)
{
  uint8_t outbuffer[1] = {0};
  outbuffer[0] = 0x40 | (temp << 4);
  HAL_I2C_Master_Transmit(i2c, MS5607_ADDR << 1, outbuffer, 1, 100);
}


/******************
 * @brief Reads the ADC value from the MS5607
 * @param i2c: pointer to the I2C_HandleTypeDef struct
 * @return: the ADC value
*/
uint32_t MS5607_ReadADC(I2C_HandleTypeDef* i2c)
{
  uint8_t outbuffer[1] = {0};
  uint8_t inbuffer[4] = {0};
  outbuffer[0] = 0;
  HAL_I2C_Master_Transmit(i2c, MS5607_ADDR << 1, outbuffer, 1, 100);
  HAL_I2C_Master_Receive(i2c,  MS5607_ADDR << 1, inbuffer, 3, 100);
  return ((inbuffer[0] << 16) | (inbuffer[1] << 8) | inbuffer[2]);
}


/*******************
 * @brief Reads the calibration coefficient from the MS5607
 * @param i2c: pointer to the I2C_HandleTypeDef struct
 * @param coeff_number: the coefficient number to read
 * @return: the coefficient value
*/
uint16_t MS5607_ReadCoeff(I2C_HandleTypeDef* i2c, uint8_t coeff_number)
{
  uint8_t outbuffer[1] = {0};
  uint8_t inbuffer[2] = {0};
  outbuffer[0] = 0xa0 | (coeff_number << 1); // read coefficient
  HAL_I2C_Master_Transmit(i2c, MS5607_ADDR << 1, outbuffer, 1, 100);
  HAL_I2C_Master_Receive(i2c,  MS5607_ADDR << 1, inbuffer, 2, 100);
  return ((inbuffer[0] << 8) | inbuffer[1]);
}


/********************
 * @brief Gets the Calibration Coefficients from the MS5607
 * @param i2c: pointer to the I2C_HandleTypeDef struct
 * @param ms: pointer to the sMS5607 struct
*/
void MS5607_GetCoefficients(I2C_HandleTypeDef* i2c, struct sMS5607* ms)
{
  int i;
  for (i=1;i<7;i++)
    ms->C[i] = MS5607_ReadCoeff(i2c, i);
}


/********************
 * @brief Gets the Temperature and Pressure from the MS5607
 * @param i2c: pointer to the I2C_HandleTypeDef struct
 * @param ms: pointer to the sMS5607 struct
 * @param temp: pointer to the temperature variable
 * @param pressure: pointer to the pressure variable
*/
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


/********************
 * @brief initailizes the MS5607 structure to all zeros
 * @param ms: pointer to the sMS5607 struct
*/
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
