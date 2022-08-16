/*
handeler for the SD card
mjeffers@ucar.edu
*/


#include "stm32f4xx_hal.h"
#include "MCU_Init.h"
#include <stdbool.h>
#include "ff.h"
uint8_t SD_FileSystem_Create(void);
uint8_t SD_Append_String_File(char *FileName, char *data, uint16_t len);
uint8_t SD_Append_Data_File(char *FileName, uint8_t *data, size_t len);
uint8_t SD_GetFiles(FILINFO *fno);
uint8_t SD_Make_File(char *FileName);
void get_random_string(char* NumString);
void get_dateString(char* dateString);
uint8_t SD_FileSystem_Create(void);
uint8_t SD_Make_Dir(char *DirName);
void BuildPath(char *path);
char *strmalloc(char *str);
uint8_t SD_File_Exists(char *FileName);
