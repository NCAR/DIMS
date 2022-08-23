/*
 * TMP117.c
 *
 *  Created on: Aug 8, 2021
 *      Author: damonb
 */

/*
*/

#include "TMP117.h"
#include "main.h"

void TMP117_InitStruct(struct sTMP117* s, I2C_HandleTypeDef* interface, uint8_t addpin)
{
  /* addpin values define what the address pin is tied to:
   * 0x00 ground
   * 0x01 V+
   * 0x10 SDA
   * 0x11 SCL
   */
  uint8_t i;
  s->Address = 0b1001000 | addpin;
  s->Interface = interface;
  s->Temperature = 0;
  s->Configured = false;
  s->Samples = 16;
  s->Index = 0;
  for (i=0;i<64;i++)
    s->Data[i] = 0;

}

void TMP117_Configure(struct sTMP117* s)
{
  uint8_t buffer[3] = {0};
  buffer[0] = 1;
  buffer[2] = 1 << 5;
  HAL_I2C_Master_Transmit(s->Interface, (s->Address) << 1, buffer, 3, 10);
  s->Configured = true;
}

void TMP117_GetTemperature(struct sTMP117* s)
{
  uint8_t i;
  uint16_t u;
  double t = 0;
  uint8_t buffer[2] = {0};
  uint8_t b2[3] = {0};
  HAL_StatusTypeDef res;
  res = HAL_I2C_Master_Transmit(s->Interface, (s->Address) << 1, buffer, 1, 10);
  if (res != HAL_OK)
    printf("Fail1\n");
  res = HAL_I2C_Master_Receive(s->Interface, (s->Address) << 1, buffer, 2, 10);
  if (res != HAL_OK)
    printf("Fail2\n");

  if ((buffer[0] == 0) && (buffer[1] == 0))
  {
    printf("Corrupt data, delaying\n");
    HAL_Delay(100);
    b2[0]=1; b2[2]=2;
    res = HAL_I2C_Master_Transmit(s->Interface, (s->Address) << 1, b2, 3, 10);
    if (res != HAL_OK)
      printf("Fail3\n");
    HAL_Delay(10);

    I2C2_State(false);
    HAL_Delay(100);
    I2C2_State(true);
    HAL_Delay(100);

    TMP117_Configure(s);

    buffer[0] = 0xe0;
  }
  else
  {

    s->Data[s->Index] = (buffer[0] << 8) | buffer[1];
    s->Index = (s->Index + 1) % (s->Samples);
    for (i=0;i<(s->Samples); i++)
    {
      u = *(&(s->Data[i]));
      t += ((double)u) * 0.0078125;
    }
    s->Temperature = t / s->Samples;
  }
}



void I2C2_State(bool s)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (s)
  {
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
    HAL_I2C_DeInit(&hi2c2);
}
