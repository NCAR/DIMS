/*
 * sdfs.h
 *
 *  Created on: Jan 11, 2021
 *      Author: damonb
 */

#ifndef INC_SDFS_H_
#define INC_SDFS_H_

#include "defs.h"
#include "fatfs.h"
#include "HR4000.h"

uint8_t SDFS_WriteString(struct sState *state, uint8_t* buffer, char* filename);
uint8_t SDFS_WriteSpectraBinary(struct sState *state, struct sSpectra *spectra, struct sGPSFrame *gps);
uint8_t SDFS_WriteSpectra(struct sState *state, struct sSpectra *spectra, struct sGPSFrame *gps);
uint8_t SDFS_WriteEnvironmental(struct sState *state, struct sGPSFrame *gps);
uint8_t SDFS_WriteCoords(struct sState *State, struct sGPSFrame *gps);
uint8_t SDFS_MakeDir(char path[]);
void SDFS_SetupFS(struct sState *state, struct sGPSFrame *gps);
void SDFS_IncrementDirectory(struct sState *state);
uint8_t SDFS_Mount();
void SDFS_PowerCycle();
void SDFS_FindNewDirectory(struct sState *state);

#endif /* INC_SDFS_H_ */
