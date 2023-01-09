/*
 * ISRegisters.c
 *
 *  Created on: Apr 21, 2021
 *      Author: damonb
 *     Edited by mjeffers
 */

#include "defs.h"
#include "GPS.h"
#include "MS5607.h"


/*******************
 * @brief Initialize the registers to 0
 * @param reg: pointer to the register array
 * @param backreg: pointer to the backup register array
*/
void InitRegisters(uint32_t* reg, uint32_t* backreg)
{
  int i;
  for (i=0; i<256; i++)
  {
    reg[i] = 0;
    backreg[i] = 0;
  }
}


/*******************
 * @brief Load the registers with the current values from the Different Structures
 * @param reg: pointer to the register array
 * @param backreg: pointer to the backup register array
 * @param usebackupreg: pointer to the usebackupreg flag
 * @param gps: pointer to the GPS Frame struct
 * @param ms: pointer to the MS5607 struct
 * @param state: pointer to the state struct of the System
*/
void LoadRegisters(uint32_t* reg, uint32_t* backreg, bool* usebackupreg, struct sGPSFrame* gps, struct sMS5607* ms, struct sState* state)
{
  int i;
  for (i=0; i<256; i++)
    backreg[i] = reg[i];

  *usebackupreg = true;

  reg[0x00] = state->TestBuffer;

  reg[0x02] = gps->Latitude * 1000000;
  reg[0x03] = gps->Longitude * 1000000;
  reg[0x04] = gps->Altitude * 100;
  reg[0x05] = gps->NumSats;
  reg[0x06] = gps->Time * 100 + gps->Ticks;
  reg[0x07] = gps->Date;

  reg[0x0e] = state->Heaters->HeaterDwell[0];
  reg[0x0f] = state->Heaters->HeaterDwell[1];
  reg[0x10] = state->Heaters->HeaterDwell[2];
  reg[0x11] = state->Heaters->HeaterDwell[3];
  reg[0x12] = state->Heaters->HeaterDwell[4];
  reg[0x13] = state->Heaters->HeaterDwell[5];
  reg[0x14] = state->TemperatureTarget[0];
  reg[0x15] = state->TemperatureTarget[1];
  reg[0x16] = state->TemperatureTarget[2];
  reg[0x17] = state->TemperatureTarget[3];
  reg[0x18] = state->TemperatureTarget[4];
  reg[0x19] = state->TemperatureTarget[5];
  reg[0x1a] = (uint32_t) state->MSPressure;
  reg[0x1b] = (uint32_t) state->MSTemperature;

  reg[0x20] =  state->HeaterControllerMode[0] |
              (state->HeaterControllerMode[1] << 1) |
              (state->HeaterControllerMode[2] << 2) |
              (state->HeaterControllerMode[3] << 3) |
              (state->HeaterControllerMode[4] << 4) |
              (state->HeaterControllerMode[5] << 5);


  *usebackupreg = false;
}


/************************
 * @brief Process the I2C Command and set the Systems States accordingly
 * @param packet: pointer to the I2C packet
 * @param state: pointer to the state struct of the System
 * @retval None
*/
void ProcessI2CCommand(uint8_t* packet, struct sState* state)
{
  uint32_t val = (packet[1] << 24) | (packet[2] << 16) | (packet[3] << 8) | packet[4];
  if (packet[0] == 0x81)
    state->TemperatureTarget[0] = val;
  if (packet[0] == 0x82)
    state->TemperatureTarget[1] = val;
  if (packet[0] == 0x83)
    state->TemperatureTarget[2] = val;
  if (packet[0] == 0x84)
    state->TemperatureTarget[3] = val;
  if (packet[0] == 0x85)
    state->TemperatureTarget[4] = val;
  if (packet[0] == 0x86)
    state->TemperatureTarget[5] = val;
  if (packet[0] == 0xff)
    state->TestBuffer = val;
}
