/*
 * ISRegisters.h
 *
 *  Created on: Apr 21, 2021
 *      Author: damonb
 */

#ifndef INC_ISREGISTERS_H_
#define INC_ISREGISTERS_H_

uint32_t Registers[255];
uint32_t BackupRegisters[255];

void InitRegisters(uint32_t* reg, uint32_t* backreg);
void LoadRegisters(uint32_t* reg, uint32_t* backreg, bool* usebackupreg, struct sGPSFrame* gps, struct sMS5607* ms, struct sState* state);
void ProcessI2CCommand(uint8_t* packet, struct sState* state);


#endif /* INC_ISREGISTERS_H_ */
