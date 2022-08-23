/*
 * AT30TS74.c
 *
 *  Created on: Aug 8, 2021
 *      Author: damonb
 */

/*
*/

#include "AT30TS74.h"

void AT30TS74_InitStruct(struct sAT30TS74* s, I2C_HandleTypeDef* interface, uint8_t add)
{
  uint8_t i;
  s->Address = 0b1001000 | add;
  s->Interface = interface;
  s->Temperature = 0;
  s->Configured = false;
  for (i=0;i<64;i++)
    s->Data[i] = 0;
}

void AT30TS74_Configure(struct sAT30TS74* s)
{
  uint8_t buffer [3] = {0};
  buffer[0] = 1; // configuration register
  buffer[1] = 0b01100000;
  HAL_I2C_Master_Transmit(s->Interface, (s->Address) << 1, buffer, 3, 10);
  s->Configured = true;
  s->Index = 0;
  s->Samples = 4;
}


void AT30TS74_GetTemperature(struct sAT30TS74* s)
{
  uint8_t i;
  uint16_t u;
  double t = 0;
  uint8_t buffer[2] = {0};
  HAL_StatusTypeDef res;
  HAL_I2C_Master_Transmit(s->Interface, (s->Address) << 1, buffer, 1, 10);
  res = HAL_I2C_Master_Receive(s->Interface, ((s->Address) << 1), buffer, 2, 10);
  if (res == HAL_OK)
  {
    s->Data[s->Index] = (buffer[0] << 8) | buffer[1];
    s->Index = (s->Index + 1) % (s->Samples);
    for (i=0;i<(s->Samples); i++)
    {
      u = *(&(s->Data[i]));
      t += ((double) u) * 0.0625 / 16;
    }
    s->Temperature = t / s->Samples;
  }
  else
  {
    s->Temperature = 99;
    printf("Could not read AT30 at address %d\n",s->Address);
  }
}



