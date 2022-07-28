#include "D_XCAM.h"
#include "TaskMonitor.h"
#include "EPS.h"
#include "SD_handler.h"
#include "TaskMonitor.h"
#include "Logging.h"
#include "stm32f4xx_hal.h"
#include "EPS.h"

uint8_t Make_HouseKeeping(const char *FileName);
uint8_t Setup_SD(void);
void main_imaging_loop(void);
uint8_t Initialize_XCAM(void);
uint8_t Write_EntireImage_SD(void);
