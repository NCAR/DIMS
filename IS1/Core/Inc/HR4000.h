/*
 * HR4000.h
 *
 *  Created on: Dec 29, 2020
 *      Author: damonb
 */

#ifndef INC_HR4000_H_
#define INC_HR4000_H_

#include "defs.h"
#include "gps.h"

extern volatile uint8_t hrbuf[8200];

struct sHR4000
{
	UART_HandleTypeDef *UART;
  uint16_t IntegrationTime_ms;
  uint16_t NextIntegrationTime_ms;
	uint8_t Summing;
	uint8_t Smoothing;
	uint16_t Delay;
	bool Checksum;
	bool DiscardNextSpectra;
	bool GetSpectra;
	bool DataReady;
	uint16_t ElapsedTime;
	uint8_t State;
};

//struct sHR4000 HR_InitStruct(void);

struct sSpectra
{
	uint8_t DataSize;
	uint8_t NumberofScans;
	uint32_t IntegrationTime_us;
	uint8_t PixelMode;
	uint8_t RawData[8192];
	uint8_t DataHeader[14];
	bool ReadyToSave;
};

void HR_ClearBuffer(UART_HandleTypeDef *huart);
uint8_t HR_SetBinaryMode(UART_HandleTypeDef *huart);
uint8_t HR_SendCommand(UART_HandleTypeDef *huart, uint8_t *command, size_t length, uint16_t wait_time);
uint8_t HR_SetIntegrationTime(UART_HandleTypeDef *huart, uint16_t time_ms);
uint8_t HR_SetSumming(UART_HandleTypeDef *huart, uint8_t count);
uint8_t HR_SetSmoothing(UART_HandleTypeDef *huart, uint8_t count);
uint8_t HR_ClearMemory(UART_HandleTypeDef *huart);
uint8_t HR_SetChecksumMode(UART_HandleTypeDef *huart, bool state);
uint8_t HR_GetSpectra(struct sHR4000* SR, struct sSpectra *Spectra);
uint8_t HR_SetTriggerMode(UART_HandleTypeDef *huart, uint8_t mode);
uint8_t HR_TriggerSpectra(UART_HandleTypeDef *huart);
uint8_t HR_AnalyzeSpectra(struct sSpectra *spectra);
uint8_t HR_Execute(struct sState *state, struct sHR4000 *HR, struct sSpectra *Spectra, struct sGPSFrame *gps);
uint8_t HR_ValidateData(struct sHR4000 *HR, struct sSpectra *Spectra);

void HR_InitStruct(struct sHR4000 *HR4000);


#endif /* INC_HR4000_H_ */
