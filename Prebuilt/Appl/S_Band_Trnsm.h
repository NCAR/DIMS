/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file S_Band_Trnsm.h
* @brief Header of S_Band_Trnsm.c
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Vassil Milev
* @version           1.0.0
* @date              2019.08.15
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.08.15, author Vassil Milev, Initial revision }
* @revision{         1.0.1  , 2019.12.02, author Vassil Milev, Added Read and Write commands for all Attenuations }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef S_BAND_TRNSM_H
#define S_BAND_TRNSM_H

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "main.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define S_BAND_TRNSM_STRCT_SIZE       (14)     /* size of one packet without the data and the CRC */
#define S_BAND_TRNSM_TX_BUFF_SIZE     (1536)
#define S_BAND_TRNSM_RX_BUFF_SIZE     (1536)
#define S_BAND_ID_LENGTH              (4)      /* Length of the S-Band identifier */
#define S_BAND_ID_RANGE_MIN           (0x2000) /* Length of the S-Band identifier */
#define S_BAND_ID_RANGE_MAX           (0x2FFF) /* Length of the S-Band identifier */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef struct{
    uint32_t Header;
    uint16_t ModuleID;
    uint16_t Length;
    uint16_t Responce;
    uint16_t CMD;
    uint16_t Type;
}__packed S_BAND_TRNSM_Pack_struct;

/*---------------------------------------------------------------------------------------
Message 0x0049 - Tranciver Chip AD9364 All Tranciver parameters.
-----------------------------------------------------------------------------------------
 Description: Message to XTR to write data to the AD9364
 Total payload: 12 bytes
 Structure of the payload: ALL TRANCIVER PARAMS
    Byte 0   |  1 byte    |  Symbol_Rate
    Byte 1   |  1 byte    |  TX_Power
    Byte 2   |  1 byte    |  Modulation_Code
    Byte 3   |  1 byte    |  Roll_Off
    Byte 4   |  1 byte    |  Pilot_Signal_On_Off
    Byte 5   |  1 byte    |  FEC_Frame_Size
    Byte 6   |  2 byte    |  Pre_Trans_Staff_Delay
    Byte 8   |  4 byte    |  Central_Frequency
*/
typedef struct{
    uint8_t  SymbolRate;
    uint8_t  TxPower;
    uint8_t  MODCOD;
    uint8_t  RollOff;
    uint8_t  PilotSygnal;
    uint8_t  FEC_Frame_Size;
    uint16_t PreTxStaffingDelay;
    float    CentralFreq;
}__packed S_BAND_TRNSM_GET_AllParams_struct;

/*---------------------------------------------------------------------------------------
Message 0x004E - Module Status Mixed Report.
-----------------------------------------------------------------------------------------
 Description: Message to Master Device Reporting Status of XTR
 Total payload: 12 bytes
 Structure of the payload: ALL TRANCIVER PARAMS
    Byte 0   |  1 byte    |  Sysyem_State
    Byte 1   |  1 byte    |  Status_Flags
    Byte 2   |  2 byte    |  Reserved
    Byte 4   |  4 byte    |  CPU_Temperature
    Byte 8   |  4 byte    |  FW_Version
*/

typedef struct{
    uint8_t  SystemState;
    uint8_t  StatusFlag;
    uint16_t Reserved;
    float    CPU_Temp;
    uint32_t FW_Version;
}__packed S_BAND_TRNSM_GET_SympleReport_struct;

typedef struct{
    float Attenuation27dBmF8025_8100Mhz;
    float Attenuation28dBmF8025_8100Mhz;
    float Attenuation29dBmF8025_8100Mhz;
    float Attenuation30dBmF8025_8100Mhz;
    float Attenuation31dBmF8025_8100Mhz;
    float Attenuation32dBmF8025_8100Mhz;
    float Attenuation33dBmF8025_8100Mhz;

    float Attenuation27dBmF8100_8200Mhz;
    float Attenuation28dBmF8100_8200Mhz;
    float Attenuation29dBmF8100_8200Mhz;
    float Attenuation30dBmF8100_8200Mhz;
    float Attenuation31dBmF8100_8200Mhz;
    float Attenuation32dBmF8100_8200Mhz;
    float Attenuation33dBmF8100_8200Mhz;

    float Attenuation27dBmF8200_8300Mhz;
    float Attenuation28dBmF8200_8300Mhz;
    float Attenuation29dBmF8200_8300Mhz;
    float Attenuation30dBmF8200_8300Mhz;
    float Attenuation31dBmF8200_8300Mhz;
    float Attenuation32dBmF8200_8300Mhz;
    float Attenuation33dBmF8200_8300Mhz;

    float Attenuation27dBmF8300_8400Mhz;
    float Attenuation28dBmF8300_8400Mhz;
    float Attenuation29dBmF8300_8400Mhz;
    float Attenuation30dBmF8300_8400Mhz;
    float Attenuation31dBmF8300_8400Mhz;
    float Attenuation32dBmF8300_8400Mhz;
    float Attenuation33dBmF8300_8400Mhz;
}__packed S_BAND_TRNSM_GET_AttenuationParams_struct;


typedef struct{
    uint8_t  DirNextAvailable;
    uint16_t NumberFiles;
    uint8_t  * FileList;
}__packed S_BAND_TRNSM_FileList_struct;

typedef struct{
    uint32_t  Size;          /* size of the file */
    uint8_t  NameLength;    /* length of the name of the file */
    uint8_t  FileName[31];  /* String with the name of the file */
}S_BAND_TRNSM_FileInfo_struct;

typedef struct{
    uint16_t  Reserved;
    uint16_t  Size;
    int32_t   FileHandler;
    uint32_t  PacketNumber;
    uint8_t   Data[S_BAND_TRNSM_TX_BUFF_SIZE];
}__packed S_BAND_TRNSM_WriteFile_struct;

typedef struct{
    uint8_t  Reserved;
    uint8_t  Status;
    uint16_t  Length;
    uint32_t  PacketNumber;
    uint8_t   Data[S_BAND_TRNSM_TX_BUFF_SIZE];
}__packed S_BAND_TRNSM_ReadFile_struct;

typedef enum{
    S_BAND_TRNSM_CMD_GET,               /* Get status */
    S_BAND_TRNSM_CMD_SET,               /* Set status */
    S_BAND_TRNSM_CMD_DIR_F,             /* Dir list of files in the SD Card */
    S_BAND_TRNSM_CMD_DIR_NEXT_F,        /* Get next page from the list of file after cmd DIR */
    S_BAND_TRNSM_CMD_DELL_F,            /* Delete a file */
    S_BAND_TRNSM_CMD_DELL_ALL_F,        /* Delete all files */
    S_BAND_TRNSM_CMD_CREATE_F,          /* Create a file */
    S_BAND_TRNSM_CMD_WRITE_F,           /* Write data to a file */
    S_BAND_TRNSM_CMD_OPEN_F,            /* Open a file */
    S_BAND_TRNSM_CMD_READ_F,            /* Read data from a file */
    S_BAND_TRNSM_CMD_SEND_F,            /* Send a file by the S-Band RF transmitter */
    S_BAND_TRNSM_CMD_TX_MODE,           /* Enter Transmit mode (High power consumption) */
    S_BAND_TRNSM_CMD_LOAD_MODE,         /* Exit Transmit mode (Low power consumption) */
    S_BAND_TRNSM_CMD_UPDATE_FW,         /* Update the firmware from a file */
    S_BAND_TRNSM_CMD_SAFE_SHUTDOWN,     /* Terminate all pending operations and prepare for shut down (Shut down must be done 1 second after that command) */
    S_BAND_TRNSM_CMD_GET_RES,           /* Pool the result from the last command */
    S_BAND_TRNSM_CMD_NUMBER             /* Get the number of all command */
}S_BAND_TRNSM_CMD_enum;

/* Values of all commands */
typedef enum{
    S_BAND_TRNSM_CMD_VAL_GET           = 0x0100,
    S_BAND_TRNSM_CMD_VAL_SET           = 0x0101,
    S_BAND_TRNSM_CMD_VAL_DIR_F         = 0x0102,
    S_BAND_TRNSM_CMD_VAL_DIR_NEXT_F    = 0x0103,
    S_BAND_TRNSM_CMD_VAL_DELL_F        = 0x0104,
    S_BAND_TRNSM_CMD_VAL_DELL_ALL_F    = 0x0105,
    S_BAND_TRNSM_CMD_VAL_CREATE_F      = 0x0106,
    S_BAND_TRNSM_CMD_VAL_WRITE_F       = 0x0107,
    S_BAND_TRNSM_CMD_VAL_OPEN_F        = 0x0108,
    S_BAND_TRNSM_CMD_VAL_READ_F        = 0x0109,
    S_BAND_TRNSM_CMD_VAL_SEND_F        = 0x010A,
    S_BAND_TRNSM_CMD_VAL_TX_MODE       = 0x0110,
    S_BAND_TRNSM_CMD_VAL_LOAD_MODE     = 0x0111,
    S_BAND_TRNSM_CMD_VAL_UPDATE_FW     = 0x0112,
    S_BAND_TRNSM_CMD_VAL_SAFE_SHUTDOWN = 0x0113,
    S_BAND_TRNSM_CMD_VAL_GET_RESULT    = 0x0114,
    S_BAND_TRNSM_CMD_VAL_NUMBER
}S_BAND_TRNSM_CMD_VAL_enum;


/* Available types for CMD "Get Result" ( 0x0101 ) */
typedef enum{
    S_BAND_TRNSM_RET_RES_GET           = S_BAND_TRNSM_CMD_VAL_GET        ,
    S_BAND_TRNSM_RET_RES_SET           = S_BAND_TRNSM_CMD_VAL_SET        ,
    S_BAND_TRNSM_RET_RES_DIR_F         = S_BAND_TRNSM_CMD_VAL_DIR_F      ,
    S_BAND_TRNSM_RET_RES_DIR_NEXT_F    = S_BAND_TRNSM_CMD_VAL_DIR_NEXT_F ,
    S_BAND_TRNSM_RET_RES_DELL_F        = S_BAND_TRNSM_CMD_VAL_DELL_F     ,
    S_BAND_TRNSM_RET_RES_DELL_ALL_F    = S_BAND_TRNSM_CMD_VAL_DELL_ALL_F ,
    S_BAND_TRNSM_RET_RES_CREATE_F      = S_BAND_TRNSM_CMD_VAL_CREATE_F   ,
    S_BAND_TRNSM_RET_RES_WRITE_F       = S_BAND_TRNSM_CMD_VAL_WRITE_F    ,
    S_BAND_TRNSM_RET_RES_OPEN_F        = S_BAND_TRNSM_CMD_VAL_OPEN_F     ,
    S_BAND_TRNSM_RET_RES_READ_F        = S_BAND_TRNSM_CMD_VAL_READ_F     ,
    S_BAND_TRNSM_RET_RES_SEND_F        = S_BAND_TRNSM_CMD_VAL_SEND_F     ,
    S_BAND_TRNSM_RET_RES_TX_MODE       = S_BAND_TRNSM_CMD_VAL_TX_MODE    ,
    S_BAND_TRNSM_RET_RES_LOAD_MODE     = S_BAND_TRNSM_CMD_VAL_LOAD_MODE  ,
    S_BAND_TRNSM_RET_RES_UPDATE_FW     = S_BAND_TRNSM_CMD_VAL_UPDATE_FW  ,
    S_BAND_TRNSM_RET_RES_NUMBER
}S_BAND_TRNSM_RetRes_enum;

/* Types for all commands that don't have type */
#define S_BAND_TRNSM_NULL_TYPE          ((uint16_t)(0x0000))

/* Available types for CMD "GET" ( 0x0100 ) */
typedef enum{
    S_BAND_TRNSM_GET_TYPE_SYMBOL_RATE     = 0x0040,
    S_BAND_TRNSM_GET_TYPE_TX_POWER        = 0x0041,
    S_BAND_TRNSM_GET_TYPE_CENTRAL_FREQ    = 0x0042,
    S_BAND_TRNSM_GET_TYPE_MODCOD          = 0x0043,
    S_BAND_TRNSM_GET_TYPE_ROLL_OFF        = 0x0044,
    S_BAND_TRNSM_GET_TYPE_PILOT_SIGNAL    = 0x0045,
    S_BAND_TRNSM_GET_TYPE_FEC_FRAME_SIZE  = 0x0046,
    S_BAND_TRNSM_GET_TYPE_PRETRASIT_DELAY = 0x0047,
    S_BAND_TRNSM_GET_TYPE_ALL_CHANGE_MODE = 0x0048,
    S_BAND_TRNSM_GET_TYPE_SIMPLE_REPORT   = 0x0049,
    //S_BAND_TRNSM_GET_TYPE_FULL_REPORT     = 0x004A, //X-Band only
    S_BAND_TRNSM_GET_TYPE_ATTENUATION_PAR = 0x004C,
    S_BAND_TRNSM_GET_TYPE_NUMBER
}S_BAND_TRNSM_GET_types_enum;

/* Available types for CMD "SET" ( 0x0101 ) */
typedef enum{
    S_BAND_TRNSM_SET_TYPE_SYMBOL_RATE     = 0x0040,
    S_BAND_TRNSM_SET_TYPE_TX_POWER        = 0x0041,
    S_BAND_TRNSM_SET_TYPE_CENTRAL_FREQ    = 0x0042,
    S_BAND_TRNSM_SET_TYPE_MODCOD          = 0x0043,
    S_BAND_TRNSM_SET_TYPE_ROLL_OFF        = 0x0044,
    S_BAND_TRNSM_SET_TYPE_PILOT_SIGNAL    = 0x0045,
    S_BAND_TRNSM_SET_TYPE_FEC_FRAME_SIZE  = 0x0046,
    S_BAND_TRNSM_SET_TYPE_PRETRASIT_DELAY = 0x0047,
    S_BAND_TRNSM_SET_TYPE_ALL_CHANGE_MODE = 0x0048,
    S_BAND_TRNSM_SET_TYPE_BAUDRATE        = 0x004B,
    S_BAND_TRNSM_SET_TYPE_NUMBER
}S_BAND_TRNSM_SET_types_enum;

/* Available types for CMD "Send File" ( 0x010A ) */
typedef enum{
    S_BAND_TRNSM_SEND_F_TYPE_WITHOUT_ISSUES = 0x0050,
    S_BAND_TRNSM_SEND_F_TYPE_WITH_PREDST    = 0x0051,
    S_BAND_TRNSM_SEND_F_TYPE_WITH_RF_TRACT  = 0x0052,
    S_BAND_TRNSM_SEND_F_TYPE_PREDIS_AND_RF  = 0x0053,   /* Recommended */
    S_BAND_TRNSM_SEND_F_TYPE_NUMBER
}S_BAND_TRNSM_SendFile_types_enum;

/* Return statuses from sending command to S-Band transmitter */
typedef enum{
    S_BAND_TRNSM_STAT_GET_RESULT  = 0x00,
    S_BAND_TRNSM_STAT_ACK         = 0x05,
    S_BAND_TRNSM_STAT_NACK        = 0x06,
    S_BAND_TRNSM_STAT_BUSY        = 0x07,
    S_BAND_TRNSM_STAT_NCE         = 0x08,
    S_BAND_TRNSM_STAT_STACK_FULL  = 0x09,
    S_BAND_TRNSM_STAT_CTNA        = 0x0A,
    S_BAND_TRNSM_STAT_WRONG_PARAM = 0xFE,
    S_BAND_TRNSM_STAT_COMM_ERR    = 0xFF,
    S_BAND_TRNSM_STAT_NUMBER
}S_BAND_TRNSM_Responce_enum;


/* Return statuses from CMD Delete and Delete All */
typedef enum{
    S_BAND_TRNSM_DELL_OK         = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_DELL_NOT_FOUND  = 0x01,    /* The file is not found */
    S_BAND_TRNSM_DELL_CARD_ERR   = 0x03,    /* SD card Error */
    S_BAND_TRNSM_DELL_PARAMS_ERR = 0xFE,    /* Parameter Error */
    S_BAND_TRNSM_DELL_COMM_ERR   = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_DELL_NUMBER
}S_BAND_TRNSM_DellStatus_enum;

/* Return statuses from CMD Create a file */
typedef enum{
    S_BAND_TRNSM_CREATE_OK         = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_CREATE_ERR        = 0x01,    /* The file cannot be created - maybe it is already existing (should be deleted first) */
    S_BAND_TRNSM_CREATE_CARD_ERR   = 0x03,    /* SD card Error */
    S_BAND_TRNSM_CREATE_PARAMS_ERR = 0xFE,    /* Parameter Error */
    S_BAND_TRNSM_CREATE_COMM_ERR   = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_CREATE_NUMBER
}S_BAND_TRNSM_CrateStatus_enum;

/* Return statuses from CMD Open a file */
typedef enum{
    S_BAND_TRNSM_OPEN_OK         = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_OPEN_ERR        = 0x01,    /* The file cannot be open - it may not exist */
    S_BAND_TRNSM_OPEN_CARD_ERR   = 0x03,    /* SD card Error */
    S_BAND_TRNSM_OPEN_PARAMS_ERR = 0xFE,    /* Parameter Error */
    S_BAND_TRNSM_OPEN_COMM_ERR   = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_OPEN_NUMBER
}S_BAND_TRNSM_OpenStatus_enum;

/* Return statuses from CMD Send a file */
typedef enum{
    S_BAND_TRNSM_SEND_OK              = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_SEND_ERR             = 0x01,    /* The file cannot be Send - maybe it is missing */
    S_BAND_TRNSM_SEND_CARD_ERR        = 0x03,    /* SD card Error */
    S_BAND_TRNSM_SEND_FILE_COMM_ERR   = 0x04,    /* RF error */
    S_BAND_TRNSM_SEND_FILE_NOT_READY  = 0x06,    /* The file is not ready to be transmitted */
    S_BAND_TRNSM_SEND_PARAMS_ERR      = 0xFE,    /* Parameter Error */
    S_BAND_TRNSM_SEND_COMM_ERR        = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_SEND_NUMBER
}S_BAND_TRNSM_SendStatus_enum;

/* Return statuses from all commands about changing the mode of the S-Band Transceiver */
typedef enum{
    S_BAND_TRNSM_CHANG_MODE_OK              = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_CHANG_MODE_SYS_ERR         = 0x01,    /* System error */
    S_BAND_TRNSM_CHANG_MODE_BUSY            = 0x03,    /* SD card Error */
    S_BAND_TRNSM_CHANG_MODE_PARAMS_ERR      = 0xFE,    /* Parameter Error */
    S_BAND_TRNSM_CHANG_MODE_COMM_ERR        = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_CHANG_MODE_NUMBER
}S_BAND_TRNSM_ChangeMode_enum;

/* Return statuses from all CMD Firmware update */
typedef enum{
    S_BAND_TRNSM_FW_UPDATE_OK              = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_FW_UPDATE_FW_FILE_ERR     = 0x01,    /* The file is corrupted */
    S_BAND_TRNSM_FW_UPDATE_OLD_SAME_FW     = 0x02,    /* Earlier or same FW version */
    S_BAND_TRNSM_FW_UPDATE_SD_CARD_ERR     = 0x03,    /* SD Card Error */
    S_BAND_TRNSM_FW_UPDATE_UPDATE_FAILD    = 0xFC,    /* Updated started, but finished with a Error */
    S_BAND_TRNSM_FW_UPDATE_PARAMS_ERR      = 0xFE,    /* Parameter Error */
    S_BAND_TRNSM_FW_UPDATE_COMM_ERR        = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_FW_UPDATE_NUMBER
}S_BAND_TRNSM_FW_update_enum;

/* Return statuses from CMD Set and Get parameters */
typedef enum{
    S_BAND_TRNSM_PARAMS_OK         = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_PARAMS_PARAMS_ERR = 0x03,    /* Parameter Error */
    S_BAND_TRNSM_PARAMS_COMM_ERR   = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_PARAMS_NUMBER
}S_BAND_TRNSM_SetGetParams_enum;

/* Return statuses from CMD Dir and Dir Next*/
typedef enum{
    S_BAND_TRNSM_DIR_OK         = 0x00,    /* Delete complete successful */
    S_BAND_TRNSM_DIR_NO_FILES   = 0x01,    /* The files cannot be found */
    S_BAND_TRNSM_DIR_CARD_ERR   = 0x03,    /* SD card Error */
    S_BAND_TRNSM_DIR_COMM_ERR   = 0xFF,    /* Communication Error with S-Band */
    S_BAND_TRNSM_DIR_NUMBER
}S_BAND_TRNSM_Dir_enum;


typedef struct{
    S_BAND_TRNSM_CMD_VAL_enum   CMD;
    uint16_t                    tx_data_max_size;
    uint16_t                    rx_data_max_size;
}__packed S_BAND_TRNSM_CMD_INFO_TxInfo_struct;


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern uint16_t S_Band_Index;
extern uint8_t S_BAND_TRNSM_S_Band_Data[];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void S_BAND_TRNSM_Init(void);
void S_BAND_TRNSM_Task(void const * argument);
uint32_t S_BAND_TRNSM_GetBaudrate(void);

S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_SymbolRate(uint16_t Identifier, uint8_t * pSymbolRate);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_TxPower(uint16_t Identifier, uint8_t * pTxPower);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_Modcod(uint16_t Identifier, uint8_t * pModcod);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_RollOff(uint16_t Identifier, uint8_t * pRollOff);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_PilotSygnal(uint16_t Identifier, uint8_t * pPilotSygnal);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_FEC_FrameSize(uint16_t Identifier, uint8_t * pFrameSize);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_PreTransDelay(uint16_t Identifier, uint16_t * pPreTransDelay);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_Param_CentralFreq(uint16_t Identifier, float * pCentralFreq);

S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_SymbolRate(uint16_t Identifier, uint8_t * pSymbolRate);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_TxPower(uint16_t Identifier, uint8_t * pTxPower);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_Modcod(uint16_t Identifier, uint8_t * pModcod);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_RollOff(uint16_t Identifier, uint8_t * pRollOff);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_PilotSygnal(uint16_t Identifier, uint8_t * pPilotSygnal);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_FEC_FrameSize(uint16_t Identifier, uint8_t * pFrameSize);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_PreTransDelay(uint16_t Identifier, uint16_t * pPreTransDelay);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_CentralFreq(uint16_t Identifier, float * pCentralFreq);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_Param_Baudrate(uint16_t Identifier, uint8_t pBaudrate);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_AttenuationParms(uint16_t Identifier, S_BAND_TRNSM_GET_AttenuationParams_struct * Attenuation);

S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_AllParameters(uint16_t Identifier, S_BAND_TRNSM_GET_AllParams_struct * Params);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_SET_AllParameters(uint16_t Identifier, S_BAND_TRNSM_GET_AllParams_struct * Params);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_SimpleReport(uint16_t Identifier, S_BAND_TRNSM_GET_SympleReport_struct * SysStatus);
S_BAND_TRNSM_SetGetParams_enum S_BAND_TRNSM_CMD_GET_AttenuationParams(uint16_t Identifier, S_BAND_TRNSM_GET_AttenuationParams_struct * SysStatus);
S_BAND_TRNSM_Dir_enum S_BAND_TRNSM_CMD_Dir(uint16_t Identifier, S_BAND_TRNSM_FileList_struct * DirData);
S_BAND_TRNSM_Dir_enum S_BAND_TRNSM_CMD_DirNext(uint16_t Identifier, S_BAND_TRNSM_FileList_struct * Params);
uint8_t S_BAND_TRNSM_FileNameParcer(uint16_t Identifier, uint8_t * FileListBuffer, S_BAND_TRNSM_FileInfo_struct * File);
S_BAND_TRNSM_DellStatus_enum S_BAND_TRNSM_DellFile(uint16_t Identifier, uint8_t * FileName);
S_BAND_TRNSM_DellStatus_enum S_BAND_TRNSM_DellAllFile(uint16_t Identifier);
S_BAND_TRNSM_CrateStatus_enum S_BAND_TRNSM_CreateFile(uint16_t Identifier, uint8_t * FileName, uint32_t FileSize, int32_t * FileHandler);
S_BAND_TRNSM_OpenStatus_enum S_BAND_TRNSM_OpenFile(uint16_t Identifier, uint8_t * FileName, uint32_t * FileHandler, uint32_t * FileSize);
uint8_t S_BAND_TRNSM_UploadFileToSBandTrnsm(uint16_t Identifier, uint8_t * FileName);
uint8_t S_BAND_TRNSM_DownloadFileFromSBandTrnsm(uint16_t Identifier, uint8_t * FileName);
S_BAND_TRNSM_SendStatus_enum S_BAND_TRNSM_SendFile(uint16_t Identifier, uint8_t * FileName);
S_BAND_TRNSM_ChangeMode_enum S_BAND_TRNSM_StartTxMode(uint16_t Identifier);
S_BAND_TRNSM_ChangeMode_enum S_BAND_TRNSM_StartLoadMode(uint16_t Identifier);
S_BAND_TRNSM_FW_update_enum S_BAND_TRNSM_FW_Update(uint16_t Identifier, uint8_t * FileName );
S_BAND_TRNSM_ChangeMode_enum S_BAND_TRNSM_ShutDownMode(uint16_t Identifier);
void S_BAND_TRNSM_ESTTC_StartCmd(uint16_t Identifier, uint8_t Cmd, uint8_t Type, uint8_t size, uint8_t * pData);

/* Low level functions */
S_BAND_TRNSM_Responce_enum S_BAND_TRNSM_SendCMD(uint16_t Identifier, S_BAND_TRNSM_CMD_enum CMD, uint16_t CMD_Type, uint8_t * TxData, uint16_t TxDataLenght);
S_BAND_TRNSM_Responce_enum S_BAND_TRNSM_GetResult(uint16_t Identifier, S_BAND_TRNSM_CMD_enum CMD, S_BAND_TRNSM_RetRes_enum CMD_Type, uint8_t * RxData, uint16_t * RxDataLenght);
/* End of Low level functions */

#endif    /* S_BAND_TRNSM_H */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
