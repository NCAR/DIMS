#include "Logging.h"
#include "defs.h"


void HAL_Recovery_Tree(HAL_StatusTypeDef ret, bool isI2C, uint8_t attempts);
void HAL_SPI_Recovery_Tree(HAL_StatusTypeDef ret, uint8_t attempts);
void HAL_I2C_Recovery_Tree(HAL_StatusTypeDef ret, uint8_t attempts);
void HAL_Recovery_State_Busy_I2C(uint8_t attemps);
void HAL_Recovery_State_Busy_SPI(uint8_t attemps);
XCAM_Recovery_Tree(uint_8_t Status);
void Restart_System();