/*
 * HR4000.c
 * This File contains the functions to process the HR4000 data
 *  Created on: Dec 29, 2020
 *      Author: damonb
 * edited by mjeffers
 */

#include "defs.h"
#include "HR4000.h"
#include "gps.h"
#include "sdfs.h"
#define vdebug (true)

#define HRDATASIZE (14 + 3840 * 2 + 2 + 461 + 1)
#define UPPER_BRIGHTNESS_LEVEL (14000)
#define PERCENT_HOT_PIXELS (0.1)
#define AUTO_ADJUST_INTEGRATION_TIME (true)

/*
 * we set up a state machine for the HR4000 with the following states:
 * 0 - uninit, setting binary mode
 * 1 - setting trigger mode
 * 2 - setting smoothing value
 * 3 - set summing value
 * 4 - change trigger mode
 * 5 - set integration time, flag next spectra to be discarded
 * 6 - if GetSpectra is set, send trigger and enable interrupt for receiving data
 * 7 - timeout if we exceed 10 seconds, flag data for saving if valid
 *
 * If the integration time is changed, move to state 5. If there's an error, move to state 0.
 */


/*******
 * @brief Executes the HR4000 state machine
 * @param state: pointer to the state struct
 * @param HR: pointer to the HR4000 struct
 * @param Spectra: pointer to the Spectra struct
 * @param gps: pointer to the GPS frame struct
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_Execute(struct sState *state, struct sHR4000 *HR, struct sSpectra *Spectra, struct sGPSFrame *gps)
{
  
  uint8_t buffer[100];
  //if the HR4000 needs to Delay, do so
  if (HR->Delay)
  {
    HR->Delay--;
    return 0;
  }

  if (HR->State < 5)
    HR->GetSpectra = false;

  if ((HR->State > 5) && (HR->NextIntegrationTime_ms != HR->IntegrationTime_ms))
    HR->State = 5;

  switch (HR->State)
  {
    case 0:
      HAL_UART_Receive(HR->UART, buffer, 20, 1);
      printf("Case 0.\n");
      if (HR_SetBinaryMode(HR->UART))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      else
      {
        HR->Delay = 20;
        HR->State++;
      }
      break;
    case 1:
      if (HR_SetTriggerMode(HR->UART, 1))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      else
      {
        HR->Delay = 20;
        HR->State++;
      }
      break;
    case 2:
      if (HR_SetSmoothing(HR->UART, HR->Smoothing))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      else
      {
        HR->Delay = 20;
        HR->State++;
      }
      break;
    case 3:
      if (HR_SetSumming(HR->UART, HR->Summing))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      else
      {
        HR->Delay = 20;
        HR->State++;
      }
      break;
    case 4:
      if (HR_SetTriggerMode(HR->UART, 0))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      else
      {
        HR->Delay = 20;
        HR->State++;
      }
      break;
    case 5:
      HR->IntegrationTime_ms = HR->NextIntegrationTime_ms;
      if (HR_SetIntegrationTime(HR->UART, HR->IntegrationTime_ms))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      HR->GetSpectra = true;
      HR->DiscardNextSpectra = true;
      HR->State++;
      break;
    case 6:
      if (HR->GetSpectra == false)
        return 0;
      HAL_StatusTypeDef result;
      if (HR_TriggerSpectra(HR->UART))
      {
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }
      HR->DataReady = false;
      result =  HAL_UART_Receive_IT(HR->UART, Spectra->RawData, HRDATASIZE);
      HR->ElapsedTime = 0;
      HR->State++;
      HR->GetSpectra = false;
      break;
    case 7:
      if ((HR->DataReady == false) && (HR->ElapsedTime < 100)) // elapsed time is units of 0.1 s  or maybe not
      {
        HR->Delay = 10;
        HR->ElapsedTime++;
        return 0;
      }
      if (HR->ElapsedTime >= 1000)
      {
        printf("Calling AbortReceive.\n");
        HAL_UART_AbortReceive_IT(HR->UART);
        HR->Delay = 300;
        HR->State = 0;
        return 1;
      }

      HR->DataReady = false;
      HR->State = 6;
      if (HR->DiscardNextSpectra)
      {
        printf("Discarding spectra.\n");
        HR->DiscardNextSpectra = false;
        HR->Delay = 100;
        return 0;
      }
      if (HR_ValidateData(HR, Spectra))
        return 1;
      Spectra->ReadyToSave = true;
      printf("Spectra ready to save.\n");
      HR->Delay = 1000; // wait 10 seconds before taking another spectra
      HR->State = 0; // Mitch wanted us to be tolerant of power-cycling.
      break;
    default:
      HR->State = 0;
      HR->Delay = 100;
      return 1;
      break;
  }
  return 0;
}


/************
 * @brief Checks the HR4000 data for errors
 * @param HR: pointer to the HR4000 struct
 * @param Spectra: pointer to the Spectra struct
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_ValidateData(struct sHR4000 *HR, struct sSpectra *Spectra)
{
  uint8_t i;
  for (i = 0; i < 14; i++)
    (Spectra->DataHeader)[i] = (Spectra->RawData)[i];

  /*
   WORD 0xFFFF – start of spectrum
   WORD Spectral Data Size Flag (0 → Data is WORD’s, 1 → Data is DWORD’s)
   WORD scan number ALWAYS 0
   WORD Number of scans accumulated together
   DWORD integration time in microseconds (LSW followed by MSW)
   WORD pixel mode
   WORDs if pixel mode not 0, indicates parameters passed to the Pixel Mode command (P)
   (D)WORDs spectral data – see Data Size Flag for variable size
   WORD 0xFFFD – end of spectrum
   */

  if (((Spectra->RawData[0] << 8) | Spectra->RawData[1]) != 0xffff)
  {
    printf("Invalid data start.\n");
    return 1;
  }
  else
    printf("Valid data start.\n");

  if (((Spectra->RawData[3] << 8) | Spectra->RawData[4]) == 0)
    Spectra->DataSize = 16;
  else if (((Spectra->RawData[3] << 8) | Spectra->RawData[4]) == 1)
    Spectra->DataSize = 32;
  else
  {
    printf("Invalid data size.\n");
    return 1;
  }

  Spectra->NumberofScans = (uint8_t) ((Spectra->RawData[6] << 8) | Spectra->RawData[7]);

  // datasheet claims format is LSW(MSB LSB) MSW(MSB LSB) but format is really MSW(MSB LSB) LSW(MSB LSB)
  Spectra->IntegrationTime_us = (Spectra->RawData[8] << 24) | (Spectra->RawData[9] << 16) | (Spectra->RawData[10] << 8) | Spectra->RawData[11];

  printf("Spectra received, scan count %u, integration time %lu us, data size %u.\n", Spectra->NumberofScans,
      Spectra->IntegrationTime_us, Spectra->DataSize);
  if (HR_AnalyzeSpectra(Spectra))
    if (AUTO_ADJUST_INTEGRATION_TIME)
      HR->NextIntegrationTime_ms  = 0.8 * HR->IntegrationTime_ms;
  // minimum integration time is a little less than 4 ms.
  if (HR->NextIntegrationTime_ms < 4)
    HR->NextIntegrationTime_ms = 4;
  return 0;
}

// "In binary mode alldata, except where noted, passes as 16-bit unsigned integers (WORDs)
// with the MSB followed by the LSB."

/*****************
 * @brief Initalize the HR4000 struct
 * @param HR4000: pointer to the HR4000 struct
 * @retval None
*/
void HR_InitStruct(struct sHR4000 *HR4000)
{
  HR4000->UART = &huart1;
  HR4000->IntegrationTime_ms = 150;
  HR4000->NextIntegrationTime_ms = 300;
  HR4000->Summing = 1;
  HR4000->Smoothing = 0;
  HR4000->Delay = 500; // wait before trying to talk to the spectrometer
  HR4000->Checksum = false;
  HR4000->DiscardNextSpectra = true;
  HR4000->ElapsedTime = 0;
  HR4000->State = 0;
  HR4000->GetSpectra = false;
  HR4000->DataReady = false;
}


/*****************
 * @brief Celar the Buffer on board the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @retval None
*/
void HR_ClearBuffer(UART_HandleTypeDef *huart)
{
  uint8_t result = 1;
  uint8_t buffer[20];
  if (vdebug) printf("Clearing UART buffer....\n");
  while (result)
    result = !(HAL_UART_Receive(huart, buffer, 20, 3));
}

/*****************
 * @brief Set the HR4000 to binary mode
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SetBinaryMode(UART_HandleTypeDef *huart)
{
  uint8_t resp[2];
  uint8_t command[2] = { 0 };
  command[0] = 'b';
  command[1] = 'B';
  if (vdebug) printf("Setting binary mode.");

  HAL_StatusTypeDef result;
  result = HAL_UART_Transmit(huart, command, 2, 10);
  if (result != 0)
    return 40 + result;
  result = HAL_UART_Receive(huart, resp, 2, 10); // was 100
  if (result != 0)
  {
    if (result == HAL_TIMEOUT)
    {
      if (vdebug) printf(" - TIMEOUT.\n");
      return 80 + result;
    }
  }
  if ((resp[1] == 2) || (resp[1] == 6))
  {
    if (vdebug) printf(" - ACK.\n");
    return 0;
  }
  if ((resp[1] == 3) || (resp[1] == 21))
  {
    if (vdebug) printf(" - NACK.\n");
    return 1;
  }
  return 1; // shouldn't reach this
}


/*****************
 * @brief Send a command to the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @param command: pointer to the command to send
 * @param length: length of the command
 * @param wait_time: time to wait for a response
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SendCommand(UART_HandleTypeDef *huart, uint8_t *command, size_t length, uint16_t wait_time)
{
  uint8_t resp = 0;
  HAL_StatusTypeDef result;
  if (length < 1)
    return 9;
  result = HAL_UART_Transmit(huart, command, length, 10);
  if (result != 0)
    return 40 + result;
  result = HAL_UART_Receive(huart, &resp, 1, wait_time);
  if (result != 0)
  {
    if (result == HAL_TIMEOUT)
    {
      if (vdebug) printf(" - TIMEOUT.\n");
      return 80 + result;
    }
  }
  if ((resp == 2) || (resp == 6))
  {
    if (vdebug) printf(" - ACK.\n");
    return 0;
  }
  if ((resp == 3) || (resp == 21))
  {
    if (vdebug) printf(" - NACK.\n");
    return 1;
  }
  return 1; // shouldn't reach this
}


/*****************
 * @brief Set the integration time of the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @param time_ms: integration time in ms
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SetIntegrationTime(UART_HandleTypeDef *huart, uint16_t time_ms)
{
  if (time_ms > 65000)
    return 1;
  uint8_t command[3] = { 0 };

  command[0] = 'I';
  command[1] = (time_ms >> 8) & 0xff;
  command[2] = time_ms & 0xff;
  if (vdebug) printf("Setting integration time to %u.", time_ms);
  return HR_SendCommand(huart, command, 3, 65000);
}


/*****************
 * @brief Set the scan summation of the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @param count: number of scans to sum
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SetSumming(UART_HandleTypeDef *huart, uint8_t count)
{
  if ((count < 1) | (count > 4))
    return 1;
  uint8_t command[3] = { 0 };
  command[0] = 'A';
  command[1] = 0;
  command[2] = count;
  if (vdebug) printf("Setting scan summation to %u.", count);
  return HR_SendCommand(huart, command, 3, 10);
}


/*****************
 * @brief Set the smoothing of the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @param count: number of scans to smooth
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SetSmoothing(UART_HandleTypeDef *huart, uint8_t count)
{
  if (count > 15)
    return 1;
  uint8_t command[3] = { 0 };
  command[0] = 'B';
  command[1] = 0;
  command[2] = count;
  if (vdebug) printf("Setting smoothing to %u.", count);
  return HR_SendCommand(huart, command, 3, 10);
}


/*****************
 * @brief Clear the Memory of the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_ClearMemory(UART_HandleTypeDef *huart)
{
  uint8_t command[3] = { 0 };
  command[0] = 'L';
  // datasheet says L followed by either a 0 or 1, which do the same thing
  // however, device sends ACK immediately following L, so we send one byte
  if (vdebug) printf("Clearing HR4000 memory.");
  return HR_SendCommand(huart, command, 1, 10);
}


/*****************
 * @brief Set the trigger mode of the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @param mode: trigger mode
 * 0 = Normal (Continuous)
 * 1 = external software trigger
 * 2 = external Hardware trigger with auto-integration
 * 3 = external Syncronization Trigger with auto-integration and auto-summing
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SetTriggerMode(UART_HandleTypeDef *huart, uint8_t mode)
{
  if (mode > 3)
    return 1;
  uint8_t command[3] = { 0 };
  command[0] = 'T';
  command[1] = 0;
  command[2] = mode;
  if (vdebug) printf("Setting trigger mode to %u.", mode);
  return HR_SendCommand(huart, command, 3, 10);
}

/****************************
 * @brief Trigger a spectra acquisition
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_TriggerSpectra(UART_HandleTypeDef *huart)
{
  uint8_t buffer[20];
  HAL_UART_Receive(huart, buffer, 20, 1);

  uint8_t command[1] = { 0 };
  command[0] = 'S';
  if (vdebug) printf("Requesting spectra.");
  return HR_SendCommand(huart, command, 1, 10);
}


/*****************
 * @brief Set the checksum mode of the HR4000
 * @param huart: pointer to the UART_HandleTypeDef struct
 * @param state: checksum mode
 * 0 = no checksum
 * 1 = checksum
 * @retval 0 if no error, 1 if error
*/
uint8_t HR_SetChecksumMode(UART_HandleTypeDef *huart, bool state)
{
  uint8_t command[3] = { 0 };
  command[0] = 'k';
  command[1] = 0;
  command[2] = (state == true); // probably only need = state;
  return HR_SendCommand(huart, command, 3, 10);
}

/*****************
 * @brief Analyze a Spectra From teh HR4000. Will print out the Min and Max and Average Values
 * @param spectra: pointer to the sSpectra struct
 * @retval 0 exposure good, 1 too many Hot pixels
*/
uint8_t HR_AnalyzeSpectra(struct sSpectra *spectra)
{
  uint32_t dsum = 0;
  uint16_t i, j, d, points;
  uint32_t dmax = 0;
  uint32_t dmin = 0xffffffff;

  uint16_t brightcount = 0;

  j = sizeof spectra->RawData / sizeof spectra->RawData[0];
  j = j - (j%2);

  points = 0;

  for (i=14;i<j;i+=2)
  {
    d = (spectra->RawData[i] << 8) | (spectra->RawData[i+1]);
    if (d == 0)
      continue;
    points++;
    dsum += d;
    if (d < dmin)
      dmin = d;
    if (d > dmax)
      dmax = d;
    if (d > UPPER_BRIGHTNESS_LEVEL)
      brightcount++;
  }
  printf("Spectra min %lu, max %lu, avg %lu.\n", dmin, dmax, dsum/points);

  if (brightcount > (points*PERCENT_HOT_PIXELS))
    return 1;
  return 0;
}
