/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ESTTC.h
* @brief Header of ESTTC.c.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Kolio
* @version           1.0.0
* @date              2018.07.04
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2018.07.04, author Kolio, Initial revision }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef ESTTC_H
#define ESTTC_H

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "string.h"
#include "fatfs.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define COM1    (FILE *)&huart1
#define COM4    (FILE *)&huart4
#define COM6    (FILE *)&huart6
#define COM7    (FILE *)&huart7
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
enum {
	ESTTC_CMD_ACCSEL_1_DATA 	= 0x00,
	ESTTC_CMD_ACCSEL_1_ACESS 	= 0x01,
	ESTTC_CMD_ACCSEL_2_DATA		= 0x02,
	ESTTC_CMD_ACCSEL_2_ACESS	= 0x03,
	ESTTC_CMD_MAG1_DATA			= 0x04,
	ESTTC_CMD_MGA1_ACESS		= 0x05,
	ESTTC_CMD_MAG2_DATA			= 0x06,
	ESTTC_CMD_MGA2_ACESS		= 0x07,
	ESTTC_CMD_GYR1_X_RADIO_DATA = 0x08,
	ESTTC_CMD_GYR1_X_ANGLE_DATA = 0x09,
	ESTTC_CMD_GYR1_X_AB_DATA    = 0x0A,
	ESTTC_CMD_GYR2_Y_RADIO_DATA = 0x0B,
	ESTTC_CMD_GYR2_Y_ANGLE_DATA = 0x0C,
	ESTTC_CMD_GYR2_Y_AB_DATA    = 0x0D,
	ESTTC_CMD_GYR3_Z_RADIO_DATA = 0x0E,
	ESTTC_CMD_GYR3_Z_ANGLE_DATA = 0x0F,
	ESTTC_CMD_GYR4_Z_AB_DATA    = 0x10,
	ESTTC_CMD_MAGTRK1_POWER		= 0x11,
	ESTTC_CMD_MAGTRK2_POWER		= 0x12,
	ESTTC_CMD_MAGTRK3_POWER		= 0x13,
	ESTTC_CMD_TEMP_PANEL_X_P	= 0x14,
	ESTTC_CMD_TEMP_PANEL_Y_P	= 0x15,
	ESTTC_CMD_TEMP_PANEL_Z_P	= 0x16,
	ESTTC_CMD_TEMP_PANEL_X_M	= 0x17,
	ESTTC_CMD_TEMP_PANEL_Y_M	= 0x18,
	ESTTC_CMD_TEMP_PANEL_Z_M	= 0x19,
	ESTTC_CMD_PHOTO_PANEL_1		= 0x1A,
	ESTTC_CMD_PHOTO_PANEL_2		= 0x1B,
	ESTTC_CMD_PHOTO_PANEL_3		= 0x1C,
	ESTTC_CMD_PHOTO_PANEL_4		= 0x1D,
	ESTTC_CMD_PHOTO_PANEL_5		= 0x1E,
	ESTTC_CMD_PHOTO_PANEL_6		= 0x1F,
	ESTTC_CMD_OUTPUT_CONTROL    = 0x20,

    ESTTC_CMD_GET_TIME          = 0x31,
    ESTTC_CMD_SET_TIME          = 0x32,
    ESTTC_CMD_GET_DATA          = 0x33,
    ESTTC_CMD_SET_DATA          = 0x34,
    ESTTC_CMD_UPTIME            = 0x35,

#ifdef ENABLE_OBC_ADCS
    ESTTC_CMD_SET_ADCS_TARGET       = 0x37,
    ESTTC_CMD_SET_ADCS              = 0x38,
    ESTTC_CMD_SET_ADCS_SETTINGS     = 0x39,
    ESTTC_CMD_SET_ADCS_DETERM_SETUP = 0x40,
    ESTTC_CMD_SET_ADCS_FORCE        = 0x41,
    ESTTC_CMD_GET_ADCS_STATUS       = 0x42,
#endif

    ESTTC_CMD_ANT_SETTINGS      = 0x47,
	ESTTC_CMD_RST_COUNTS        = 0x61,
	ESTTC_CMD_FAULTS_TST        = 0x62,
	ESTTC_CMD_RESET             = 0x7F,
	ESTTC_CMD_PAR_NUM           = 0x80
} ESTTC_CommandEnum;

typedef enum {
    ESTTC_COMM_INTEFACE,
    ESTTC_PAYLOAD_INTEFACE,
    ESTTC_SYSCOMM_INTEFACE,
    ESTTC_INTERFACE_NUMBER
}ESTTC_InterfacesEnum;

typedef struct{
ESTTC_InterfacesEnum cmd;
uint8_t              length;
}esttc_cmd_params_type;


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern volatile char RxBuffer[ESTTC_INTERFACE_NUMBER][UART_BUFFER_SIZE];
extern volatile uint32_t RxBuffHead[ESTTC_INTERFACE_NUMBER], RxBuffTail[ESTTC_INTERFACE_NUMBER], RxBuffLen[ESTTC_INTERFACE_NUMBER];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ESTTC_InitTask(void);
void ESTTC_CMD_CRC_Print_string(FILE * ComInterface, char * print_buff);
void ESTTC_CMD_CRC_Print_raw_data(FILE * ComInterface, char * print_buff, uint16_t size);
void ESTTC_PrintVersion(FILE *f);

#endif    /* ESTTC_H */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
