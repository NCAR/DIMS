#include "SD_Handler.h"
#include "stm32f4xx_hal.h"
#include "D_XCAM.h"
#include "TaskMonitor.h"
#include "EPS.h"




/**
 * @brief This Library Helps with Toplevel handling of SD reading and Writing for our Application.
 * @note The Max File Name Can only be 11 char without using extended libraries
 * @param: *FileName: The Path Of the File.
 * @retval: (3) if it fails to mount the SD Card
 *          (2) if it fails to open the file, 
 *          (1) if it fails to write to the file, 
 *          (0) if it successfully makes the file and writes to it.
 */



uint8_t Write_To_HK(char *String);
uint8_t Make_HouseKeeping(char *FileName);
uint8_t Setup_SD(void);
uint8_t get_next_image_id(char *ImageFileName, char *HeaderFileName);
void get_dateString(char* dateString);
uint8_t get_next_housekeeping_file_id(char *String);
void Set_Global_Variables(void);
void get_random_string(char* NumString);
uint8_t print(char *String);
