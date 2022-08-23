/*
 * funcs.h
 *
 *  Created on: Apr 14, 2021
 *      Author: damonb
 */

#ifndef INC_FUNCS_H_
#define INC_FUNCS_H_

#include "defs.h"


uint64_t GetMilliseconds(void);
void USBSendString(char* buf);
void InitDWTTimer(void);
void Delay_us(uint32_t us);
int32_t UIntToInt(uint32_t x);
uint32_t IntToUInt(int32_t x);
void InitState(struct sState* state);
void SetHeater(int heater, bool state);
int parse_comma_delimited_str(char *string, char **fields, int max_fields);

#endif /* INC_FUNCS_H_ */
