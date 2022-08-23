/*
 * defs.h
 *
 *  Created on: Apr 14, 2021
 *      Author: damonb
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_

#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;

struct sHeaters
{
  uint8_t HeaterDwell[6];
};

struct sState
{
  uint32_t TemperatureTarget[6];
  int32_t MSPressure, MSTemperature;
  bool HeaterControllerMode[6];
  uint32_t TestBuffer;
  char RootPath[100];
  char SpectraPath[100];
  uint16_t SubDirectory;
  uint8_t CreatedFiles;
  struct sHeaters* Heaters;
  uint16_t SpectraCount;

};


struct sUARTBuffer
{
  uint8_t Index;
  uint8_t ByteBuffer;
  uint8_t ReceiveBuffer[250];
  uint8_t ReadySentence[250];
};

#endif /* INC_DEFS_H_ */
