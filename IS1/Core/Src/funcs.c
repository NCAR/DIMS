/*
 * funcs.c
 * General Functions To be used in this Software
 *  Created on: Apr 14, 2021
 *      Author: damonb
 *  edited by mjeffers
 */

#include "funcs.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "defs.h"
#include <math.h>

extern volatile uint64_t ElapsedSeconds;
extern volatile uint16_t ClockTick;


/****
 * @brief Get the current time in milliseconds
 * @retval time in milliseconds
*/
uint64_t GetMilliseconds(void)
{
  uint64_t s1=0;
  uint64_t s2=1;
  uint16_t t1=0;
  uint16_t t2=1;

  while (s1 != s2)
  {
    s1=ElapsedSeconds;
    s2=ElapsedSeconds;
  }
  while (t1 != t2)
  {
    t1=ClockTick;
    t2=ClockTick;
  }
  return s1*1000+t1*10;
}

/****
 * @brief Initalize the state machine for the system
 * @param state: pointer to the state struct
 * @retval None
*/
void InitState(struct sState* state)
{
  int i;
  state->TestBuffer = 525600;

  for (i=0;i<6; i++)
  {
    state->TemperatureTarget[i] = 233000;
    state->HeaterControllerMode[i] = true;
  }
  state->MSPressure = 0;
  state->MSTemperature = 0;
  state->SpectraCount = 0;
}


/****
 * @brief Set the heater state
 * @param heater: heater number
 * @param state: true for on, false for off
 * @retval None
*/
void SetHeater(int heater, bool state)
{
  if (state)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 >> heater, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 >> heater, GPIO_PIN_RESET);
}


/*****************************************************************************
* @brief Parse a comma delimited string into an array of strings
* @param string: pointer to the string to parse
* @param fields: pointer to the array of strings
* @param max_fields: maximum number of fields to parse
* @retval number of fields parsed
******************/
int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
   int i = 0;
   fields[i++] = string;
   while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
      *string = '\0';
      fields[i++] = ++string;
   }
   return --i;
}

/**
 * @brief Send a string to the USB port
 * @param buf buffer to Send
 */
void USBSendString(char* buf)
{
  int i, res;
  for (i=0;i<50; i++)
  {
    res = CDC_Transmit_FS((uint8_t *)buf, strlen(buf));
    if (res == USBD_OK)
      return;
    HAL_Delay(1);
  }
}


/****
* @brief Initalize a DWT timer
* @retval None
*/
void InitDWTTimer(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0; // reset the counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk ; // enable the counter
}

/*****
 * @brief Delay for a number of microseconds
 * @param us: number of microseconds to delay
 * @retval None
*/
void Delay_us(uint32_t us)
{
  volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
  volatile uint32_t start = DWT->CYCCNT;
  do  {
  } while((DWT->CYCCNT - start) < cycles);
}


/***
 * @brief Convert unsigned 32 bit integer to signed 32 bit integer
 * @param x: unsigned 32 bit integer
 * @retval signed 32 bit integer
*/
int32_t UIntToInt(uint32_t x) {
    union { uint32_t u; int32_t i; } un;
    un.u = x;
    return un.i;
}

/***
 * @brief Convert signed 32 bit integer to unsigned 32 bit integer
 * @param x: signed 32 bit integer
 * @retval unsigned 32 bit integer
*/
uint32_t IntToUInt(int32_t x) {
    union { uint32_t u; int32_t i; } un;
    un.i = x;
    return un.u;
}

