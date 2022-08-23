/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ESTTC.c
* @brief EnduroSat telemetry and telecommand communication protocol
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

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "AppTasks.h"
#include "ESTTC.h"
#include "MCU_Init.h"
#include "fatfs.h"
#include "panels.h"
#include "version.h"
#include "DAT_Inputs.h"
#include "TaskMonitor.h"
#include "es_crc32.h"
#include "Svc_RTC.h"
#include "EEPROM_emul.h"
#include "AntUHF.h"
#include "S_Band_Trnsm.h"
#include "X_Band_Trnsm.h"
#include "stm32f4xx_it.h"
#include <uuid.h>
#ifdef ENABLE_OBC_ADCS
    #include <libADCS/ADCS.h>
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define ESTTS_TASK_TIME_CYCLE           (10)                                /* Time in ms, after which the task will be called again */
#define ESTTS_SYNC_FILE_TIMEOUT         ((1000/ESTTS_TASK_TIME_CYCLE)*30)   /* 30 seconds is the maximum possible time for PIPE mode of the UHF transmitter */
#define ESTTS_SYNC_FILE_PACKETS         (200)                               /* Max number of packets without synchronisation of the file with the buffers */
#define ESTTC_CYMBOLS_IN_CRC            (9)                                 /* 8 symbols for 4 bytes hex + 1 symbol "space" to separate CRC from the command */
#ifdef ENABLE_OBC_ADCS
    #define ESTTC_NUMBER_READ_CMDS          (36)                                /* Number of read commands */
    #define ESTTC_NUMBER_WRITE_CMDS         (23)                                /* Number of write commands */
#else
    #define ESTTC_NUMBER_READ_CMDS          (35)                                /* Number of read commands */
    #define ESTTC_NUMBER_WRITE_CMDS         (18)                                /* Number of write commands */
#endif
#define ESTTC_FAULT_TESTS_ENABLE

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* No Internal types definition */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/**
* @brief Arbitrary UART (COM1, COM4, COM6) ISR reception buffer
*/
volatile char RxBuffer[ESTTC_INTERFACE_NUMBER][UART_BUFFER_SIZE];
/**
* @brief Arbitrary UART (COM1, COM4, COM6) ISR reception buffer head and tail positions and length
*/
volatile uint32_t RxBuffHead[ESTTC_INTERFACE_NUMBER], RxBuffTail[ESTTC_INTERFACE_NUMBER], RxBuffLen[ESTTC_INTERFACE_NUMBER];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static char txline[ESTTC_INTERFACE_NUMBER][LINE_BUFFER_SIZE];
static char rxline[ESTTC_INTERFACE_NUMBER][LINE_BUFFER_SIZE];
static const FILE * Esttc_usart_interfaces[ESTTC_INTERFACE_NUMBER] = {COM1, COM4, COM6};
static uint32_t fpos[ESTTC_INTERFACE_NUMBER], bsize[ESTTC_INTERFACE_NUMBER];
static FILINFO fno[ESTTC_INTERFACE_NUMBER];
static uint16_t pack_data_position[ESTTC_INTERFACE_NUMBER] = {0,0,0};
static uint16_t ESTTC_sync_file_timeout[ESTTC_INTERFACE_NUMBER];        /* After starting to write a file, if the communication is stopped but the file is not closed or synchronised, it will happen automatically */
static uint16_t ESTTC_sync_file_numbPackets[ESTTC_INTERFACE_NUMBER];    /* After starting to write a file, sync the file after a certain number of packets */
static FIL df[ESTTC_INTERFACE_NUMBER];
static DIR dd[ESTTC_INTERFACE_NUMBER];
static char print_buff[ESTTC_INTERFACE_NUMBER][LINE_BUFFER_SIZE + 1 + ESTTC_CYMBOLS_IN_CRC + 1 + 1];   /* buffer just to print responses = data + Checksum + CRC + \r + \0 */

/* list of all commands and their lengths without the CRC */                           /* CMD number */                  /* Length */
static const esttc_cmd_params_type ESTTC_ReadCmdLenght[ESTTC_NUMBER_READ_CMDS] = {
                                                                                    {/* 0x00 */ ESTTC_CMD_ACCSEL_1_DATA      ,  8  },
                                                                                    {/* 0x01 */ ESTTC_CMD_ACCSEL_1_ACESS     , 10  },
                                                                                    {/* 0x02 */ ESTTC_CMD_ACCSEL_2_DATA      ,  8  },
                                                                                    {/* 0x03 */ ESTTC_CMD_ACCSEL_2_ACESS     , 10  },
                                                                                    {/* 0x04 */ ESTTC_CMD_MAG1_DATA          ,  8  },
                                                                                    {/* 0x05 */ ESTTC_CMD_MGA1_ACESS         , 10  },
                                                                                    {/* 0x06 */ ESTTC_CMD_MAG2_DATA          ,  8  },
                                                                                    {/* 0x07 */ ESTTC_CMD_MGA2_ACESS         , 10  },
                                                                                    {/* 0x08 */ ESTTC_CMD_GYR1_X_RADIO_DATA  ,  8  },
                                                                                    {/* 0x09 */ ESTTC_CMD_GYR1_X_ANGLE_DATA  ,  8  },
                                                                                    {/* 0x0A */ ESTTC_CMD_GYR1_X_AB_DATA     , 10  },
                                                                                    {/* 0x0B */ ESTTC_CMD_GYR2_Y_RADIO_DATA  ,  8  },
                                                                                    {/* 0x0C */ ESTTC_CMD_GYR2_Y_ANGLE_DATA  ,  8  },
                                                                                    {/* 0x0D */ ESTTC_CMD_GYR2_Y_AB_DATA     , 10  },
                                                                                    {/* 0x0E */ ESTTC_CMD_GYR3_Z_RADIO_DATA  ,  8  },
                                                                                    {/* 0x0F */ ESTTC_CMD_GYR3_Z_ANGLE_DATA  ,  8  },
                                                                                    {/* 0x10 */ ESTTC_CMD_GYR4_Z_AB_DATA     , 10  },
                                                                                    {/* 0x14 */ ESTTC_CMD_TEMP_PANEL_X_P     ,  8  },
                                                                                    {/* 0x15 */ ESTTC_CMD_TEMP_PANEL_Y_P     ,  8  },
                                                                                    {/* 0x16 */ ESTTC_CMD_TEMP_PANEL_Z_P     ,  8  },
                                                                                    {/* 0x17 */ ESTTC_CMD_TEMP_PANEL_X_M     ,  8  },
                                                                                    {/* 0x18 */ ESTTC_CMD_TEMP_PANEL_Y_M     ,  8  },
                                                                                    {/* 0x19 */ ESTTC_CMD_TEMP_PANEL_Z_M     ,  8  },
                                                                                    {/* 0x1A */ ESTTC_CMD_PHOTO_PANEL_1      ,  8  },
                                                                                    {/* 0x1B */ ESTTC_CMD_PHOTO_PANEL_2      ,  8  },
                                                                                    {/* 0x1C */ ESTTC_CMD_PHOTO_PANEL_3      ,  8  },
                                                                                    {/* 0x1D */ ESTTC_CMD_PHOTO_PANEL_4      ,  8  },
                                                                                    {/* 0x1E */ ESTTC_CMD_PHOTO_PANEL_5      ,  8  },
                                                                                    {/* 0x1F */ ESTTC_CMD_PHOTO_PANEL_6      ,  8  },
                                                                                    {/* 0x20 */ ESTTC_CMD_OUTPUT_CONTROL     ,  8  },
                                                                                    {/* 0x31 */ ESTTC_CMD_GET_TIME           ,  8  },
                                                                                    {/* 0x33 */ ESTTC_CMD_GET_DATA           ,  8  },
                                                                                    {/* 0x35 */ ESTTC_CMD_UPTIME             ,  8  },
#ifdef ENABLE_OBC_ADCS
                                                                                    {/* 0x42 */  ESTTC_CMD_GET_ADCS_STATUS   ,  8  },
#endif
                                                                                    {/* 0x61 */ ESTTC_CMD_RST_COUNTS         ,  8  },
                                                                                    {/* 0x7F */ ESTTC_CMD_RESET              ,  8  }
                                                                                 };

static const esttc_cmd_params_type ESTTC_WriteCmdLenght[ESTTC_NUMBER_WRITE_CMDS] = {
                                                                                    {/* 0x01 */  ESTTC_CMD_ACCSEL_1_ACESS    , 14  },
                                                                                    {/* 0x03 */  ESTTC_CMD_ACCSEL_2_ACESS    , 14  },
                                                                                    {/* 0x05 */  ESTTC_CMD_MGA1_ACESS        , 14  },
                                                                                    {/* 0x07 */  ESTTC_CMD_MGA2_ACESS        , 14  },
                                                                                    {/* 0x0A */  ESTTC_CMD_GYR1_X_AB_DATA    , 16  },
                                                                                    {/* 0x0D */  ESTTC_CMD_GYR2_Y_AB_DATA    , 16  },
                                                                                    {/* 0x10 */  ESTTC_CMD_GYR4_Z_AB_DATA    , 16  },
                                                                                    {/* 0x11 */  ESTTC_CMD_MAGTRK1_POWER     , 14  },
                                                                                    {/* 0x12 */  ESTTC_CMD_MAGTRK2_POWER     , 14  },
                                                                                    {/* 0x13 */  ESTTC_CMD_MAGTRK3_POWER     , 14  },
                                                                                    {/* 0x20 */  ESTTC_CMD_OUTPUT_CONTROL    , 14  },
                                                                                    {/* 0x32 */  ESTTC_CMD_SET_TIME          , 22  },
                                                                                    {/* 0x34 */  ESTTC_CMD_SET_DATA          , 22  },
                                                                                    {/* 0x35 */  ESTTC_CMD_UPTIME            , 26  },
#ifdef ENABLE_OBC_ADCS
                                                                                    {/* 0x37 */  ESTTC_CMD_SET_ADCS_TARGET       , 62  },
                                                                                    {/* 0x38 */  ESTTC_CMD_SET_ADCS              ,  8  },
                                                                                    {/* 0x39 */  ESTTC_CMD_SET_ADCS_SETTINGS     ,130  },
                                                                                    {/* 0x40 */  ESTTC_CMD_SET_ADCS_DETERM_SETUP ,154  },
                                                                                    {/* 0x41 */  ESTTC_CMD_SET_ADCS_FORCE        , 12  },
#endif
                                                                                    {/* 0x47 */  ESTTC_CMD_ANT_SETTINGS      , 18  },
                                                                                    {/* 0x61 */  ESTTC_CMD_RST_COUNTS        , 12  },
                                                                                    {/* 0x62 */  ESTTC_CMD_FAULTS_TST        , 12  },
                                                                                    {/* 0x7F */  ESTTC_CMD_RESET             , 12  }
                                                                                 };

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint32_t GetPhrase(char *dst, uint32_t len, char term, ESTTC_InterfacesEnum Interface);
static int ESTTC_getchar(ESTTC_InterfacesEnum Interface);
static int getbyte(uint32_t tmt_ms, ESTTC_InterfacesEnum Interface);
static uint8_t HexToBin(uint8_t hb, uint8_t lb);
static uint8_t ESTTC_ProcessData(ESTTC_InterfacesEnum Interface);
static void ESTTC_UART_TASK(void const * argument);
static uint16_t ESTTC_GetCmdLength( uint8_t cmd_type , uint8_t CMD );
static uint32_t ESTTC_ExtractCRC(char * crc_buffer);
static int ESTTC_CheckDate(int year, int month, int day );

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Initialises the ESTTC (EnduroSat Telemetry and TeleCommunications) FreeRTOS task
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ESTTC_InitTask(void)
{
    uint8_t u8_index;
    HAL_StatusTypeDef retVal;

    for( u8_index = 0; u8_index < ESTTC_INTERFACE_NUMBER ; u8_index ++ )
    {
        df[u8_index].obj.fs = 0;

        RxBuffHead[u8_index] = 0;
        RxBuffTail[u8_index] = 0;
        RxBuffLen[u8_index] = 0;

        fpos[u8_index] = 0;
        bsize[u8_index] = 0;
        ESTTC_sync_file_timeout[u8_index] = 0;
        ESTTC_sync_file_numbPackets[u8_index] = 0;
    }

    COMM = COM1;
    SYSCON = COM4;
    PAYLOAD = COM6;

    /* Start reception using interrupts through USART */
    retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)COMM, (uint8_t *)&USART_rx_data_dummy, 1);
    if( HAL_OK != retVal ){
        Error_Handler();
    }

    retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)SYSCON, (uint8_t *)&USART_rx_data_dummy, 1);
    if( HAL_OK != retVal ){
            Error_Handler();
    }

    retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)PAYLOAD, (uint8_t *)&USART_rx_data_dummy, 1);
    if( HAL_OK != retVal ){
            Error_Handler();
    }

    osThreadDef(myESTTC_UART_TASK, ESTTC_UART_TASK, osPriorityLow, 1, 6*128);
    osThreadCreate(osThread(myESTTC_UART_TASK), NULL);
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Task serving ESTTC protocol through 3 USARTs
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static void ESTTC_UART_TASK(void const * argument)
{

  uint8_t ProcessedPacket = 0;

  if( pdTRUE == InitCdCard() )
  {
      FRESULT appl_fd_result = FR_INVALID_PARAMETER;
      FILINFO  file_info;
      uint32_t i, j;
      FIL df;

      HAL_RTC_GetTime(&hrtc, &sTime, calendar_format);
      HAL_RTC_GetDate(&hrtc, &sDate, calendar_format);

      appl_fd_result = f_open(&df, "0:/sys.log", FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
      if(appl_fd_result == FR_OK)
      {
          appl_fd_result = f_stat ((char *)"0:/sys.log", &file_info);
      }else if(appl_fd_result == FR_DISK_ERR){
          // on disk error - try to reinitialise the SD card
          InitCdCard();
      }

      if( FR_OK == appl_fd_result )
      {
          appl_fd_result = f_lseek(&df, file_info.fsize);   //Go to the end of the file

          if( FR_OK == appl_fd_result )
          {
              if( FR_OK == appl_fd_result )
              {
                  i = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
                  j = *((__IO uint32_t*)MAILBOX_ADDRESS);
                  fprintf((FILE *)&df, ">%4d/%02d/%02d %2d:%02d:%02d > RTC_REG=%04X%04X MAILBOX_REG=%04X%04X",
                          sDate.Year, sDate.Month, sDate.Date,
                          sTime.Hours, sTime.Minutes, sTime.Seconds,
                          (uint16_t)(i >> 16),(uint16_t)(i),
                          (uint16_t)(j >> 16),(uint16_t)(j));

                    ESTTC_PrintVersion((FILE *)&df);
              }
              f_close(&df);
          }else{
            fprintf(PAYLOAD, "ERR+SYS.LOG(%u)\r", (uint16_t)appl_fd_result);
            fprintf(COMM   , "ERR+SYS.LOG(%u)\r", (uint16_t)appl_fd_result);
          }
      }
  }

  TaskMonitor_TaskInitialized(TASK_MONITOR_ESTTC);   /* The task is initialised and is ready */

  for(;;)
  {
      TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */

      ProcessedPacket = ProcessedPacket | ESTTC_ProcessData(ESTTC_COMM_INTEFACE);

      ProcessedPacket = ProcessedPacket | ESTTC_ProcessData(ESTTC_PAYLOAD_INTEFACE );

      ProcessedPacket = ProcessedPacket | ESTTC_ProcessData(ESTTC_SYSCOMM_INTEFACE );

	  if ( ProcessedPacket == 0 )  /* if there were no packets to process */
	  {
	      /* give time for the other processes */
	      osDelay(10);
	  }
	  ProcessedPacket = 0;
  }
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Prints a string on COM serial port
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      FILE * ComInterface - output COM serial interface, char * print_buff - null-terminated string to print out
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ESTTC_CMD_CRC_Print_string(FILE * ComInterface, char * print_buff)
{
    uint16_t print_len;

    if( print_buff != NULL )
    {
        print_len = strlen(print_buff); /* get string lengh */

        ESTTC_CMD_CRC_Print_raw_data(ComInterface, print_buff, print_len);
    }else{
        Error_Handler();
    }
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Prints arbitrary binary buffer as a hex string on COM serial port
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      FILE * ComInterface - output COM serial interface, char * print_buff - output binary buffer, uint16_t size - output binary buffer size
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ESTTC_CMD_CRC_Print_raw_data(FILE * ComInterface, char * print_buff, uint16_t size)
{
    uint16_t print_len = size;
    uint32_t CRC_value_calc;

    if( print_buff != NULL )
    {
        CRC_value_calc = crc32(0, (BYTE *)print_buff, print_len);   /* Calculate the CRC */
        sprintf(&print_buff[print_len], " %08X\r", (unsigned int)CRC_value_calc);   /* Attach the calcuated CRC at the end of the string */
        print_len += 10;    /* add to the length the new added 10 symbols, 8 for CRC, 1 interval before it and \r after it */
        (void)fwrite(print_buff, 0, print_len, ComInterface );  /* print the symbols without any formating */
    }else{
        Error_Handler();
    }
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Prints the ESTTC version on the UART/USART COM interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      FILE *f - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ESTTC_PrintVersion(FILE *f)
{
  fprintf(f, "SW version: Appl v.%d.%02d <%s %s>\r\n",
          verFW_MAJOR_REV_NB, verFW_MINOR_REV_NB, __DATE__, __TIME__);

  fprintf(f, "System UUID: %08X-%08X-%08X\r\n",
          (unsigned int)(STM32_UUID[0]), (unsigned int)(STM32_UUID[1]), (unsigned int)(STM32_UUID[2]));
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Function processing all ESTTC commands
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint8_t ESTTC_ProcessData(ESTTC_InterfacesEnum Interface)
{
    uint32_t len;
    uint32_t i, j;
    uint16_t utmp;
    uint32_t br;
    Compass_Axis_t B_raw;
    AxesRaw_t data;
    Temperature_t tmperature_sens;
#ifdef ENABLE_OBC_ADCS
    ADCS_Target_t ADCSTarget;
    ADCS_Status_t ADCSStatus[2];
#endif
    int16_t tp;
    char s8_tmp;
    FRESULT fr;
    FILE * ComInterface;
    uint8_t ProcessedPacket = 0;
    uint32_t CRC_value_calc;
    uint32_t CRC_value_rx;
    uint16_t cmd_length;

    if( Interface >= ESTTC_INTERFACE_NUMBER )
    {
        Error_Handler();
    }

    if(GetPhrase(rxline[Interface], LINE_BUFFER_SIZE-1, '\r', Interface))
    {
        char *begin;

      ComInterface = (FILE *)Esttc_usart_interfaces[Interface];

      begin = strstr(rxline[Interface], "ES+");
      if(begin != NULL)
      {
        ProcessedPacket = 1;

        len = strlen(begin);

        if( len <= LINE_BUFFER_SIZE )
        {
          strcpy(txline[Interface], begin);
          for (i = 4, j = 4; i < len; i+=2, j++)
          {
              txline[Interface][j] = HexToBin(txline[Interface][i], txline[Interface][i+1]);
          }

          if((txline[Interface][4] == OBC_I2C_ADDRESS)||(txline[Interface][4] == S_X_BAND_ADDRESS))
          {
              if (txline[Interface][4] == OBC_I2C_ADDRESS)
              {
                  if (txline[Interface][3] == 'R')
                  {
                      cmd_length = ESTTC_GetCmdLength( 0 , txline[Interface][5] );
                  }else if (txline[Interface][3] == 'W')
                  {
                      cmd_length = ESTTC_GetCmdLength( 1 , txline[Interface][5] );
                  }else if (txline[Interface][3] == 'D'){
                      cmd_length = len;
                  }else{
                      sprintf(print_buff[Interface], "ERR cmd");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      return ProcessedPacket;
                  }
              }else if(txline[Interface][4] == S_X_BAND_ADDRESS)
              {
                  if (txline[Interface][3] == 'R')
                  {
                      cmd_length = ESTTC_GetCmdLength( 2 , txline[Interface][5] );
                  }else if (txline[Interface][3] == 'W')
                  {
                      cmd_length = ESTTC_GetCmdLength( 3 , txline[Interface][5] );
                  }else if (txline[Interface][3] == 'D'){
                      cmd_length = len;
                  }else{
                      sprintf(print_buff[Interface], "ERR cmd");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      return ProcessedPacket;
                  }
              }
#ifndef SX_BAND_TESTBOARD_OBC_SAFETY_OFF
              if( len == cmd_length + ESTTC_CYMBOLS_IN_CRC )
              {
                  CRC_value_calc = crc32(0, (BYTE *)begin, len-ESTTC_CYMBOLS_IN_CRC);

                  CRC_value_rx = ESTTC_ExtractCRC(&begin[cmd_length+1]);

                  if( CRC_value_calc != CRC_value_rx )
                  {
                    sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    return ProcessedPacket;
                  }

              }else if( len != cmd_length )
              {
                  sprintf(print_buff[Interface], "Err - Invalid length of the CMD");
                  ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  return ProcessedPacket;
              }
#endif /* SX_BAND_TESTBOARD_OBC_SAFETY_OFF */
          }

          if (txline[Interface][3] == 'R')
          {
            if (txline[Interface][4] == S_X_BAND_ADDRESS)
            {
                uint16_t Identifier = (txline[Interface][6]<<8) + txline[Interface][7];

                switch( txline[Interface][5] )
                {
                    case 0x01:   // Symbol Rate
                    case 0x02:   // Tx Power
                    case 0x03:   // Central Frequency
                    case 0x04:   // MODCOD
                    case 0x05:   // Roll-Off
                    case 0x06:   // Pilot Signal On/Off
                    case 0x07:   // FEC Frame size
                    case 0x08:   // Retransmission Staf. Delay
                    case 0x09:   // All Change Mode Parameters
                    case 0x0A:   // Simple Report
                    case 0x0B:   // Full Report
#ifdef ENABLE_SX_BAND_TESTBOARD
                    case 0x0D:   // Attenuation parameters
#endif /* #ifdef ENABLE_SX_BAND_TESTBOARD */
                    case 0x40:   // Dir - all files
                    {
                        if((Identifier >= S_BAND_ID_RANGE_MIN)&&(Identifier <= S_BAND_ID_RANGE_MAX))
                        {
                            sprintf(print_buff[Interface], "Requesting S-Band CMD 0x%X ... ",txline[Interface][5]);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                            S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'R', 0, NULL);  // Read (MSB = 0) -> CMD 1 (LSB = 3)
                        }else if((Identifier >= X_BAND_ID_RANGE_MIN)&&(Identifier <= X_BAND_ID_RANGE_MAX))
                        {
                            sprintf(print_buff[Interface], "Requesting X-Band CMD 0x%X ... ",txline[Interface][5]);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                            X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'R', 0, NULL);    // Read (MSB = 0) -> CMD 1 (LSB = 3)
                        }else{
                            sprintf(print_buff[Interface], "Err - Unknown ID %X", Identifier);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }break;

                    default:
                    {
                        sprintf(print_buff[Interface], "Err - Wrong CMD");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }

            }
            else if (txline[Interface][4] == EPS_I2C_ADDRESS)
            {
                uint8_t timeout = 0;
                HAL_StatusTypeDef I2C_retStat;
                uint8_t temp_reg_buff[2];
                uint16_t eps_reg;

                cmd_length = 10;
                if( len == cmd_length + ESTTC_CYMBOLS_IN_CRC )
                {
                    CRC_value_calc = crc32(0, (BYTE *)begin, len-ESTTC_CYMBOLS_IN_CRC);

                    CRC_value_rx = ESTTC_ExtractCRC(&begin[cmd_length+1]);

                    if( CRC_value_calc != CRC_value_rx )
                    {
                      sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      return ProcessedPacket;
                    }

                }else if( len != cmd_length )
                {
                    sprintf(print_buff[Interface], "ERR - length");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    return ProcessedPacket;
                }

                //Validate the requested register number
                if((txline[Interface][6] < EPS_MAX_PAR_NUM)&&(txline[Interface][6] > 0))
                {
                    /* Read to Communication interface I2C */
                    MX_I2C1_Init(); //Enable I2C1 interface
                    do
                    {

                        if( timeout >= 1 )
                        {
                            // if there is any error reset the I2C interface
                            if(( I2C_retStat == HAL_ERROR )||( I2C_retStat == HAL_TIMEOUT)||(( I2C_retStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY)))
                            {
                                I2C_Reset(&hi2c1);
                            }
                            osDelay(5);

                            if( timeout >= 10 )
                            {
                                // Stop trying after certain times
                                break;
                            }
                        }

                        //Read the register from the EPS
                        I2C_retStat = HAL_I2C_Mem_Read(&hi2c1, EPS_I2C_ADDRESS<<1, txline[Interface][6], sizeof(uint8_t), temp_reg_buff, sizeof(uint16_t), 10);

                        timeout ++; //count one more try
                    }while( I2C_retStat != HAL_OK);
                    HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface

                    if (HAL_OK == I2C_retStat)
                    {
                        //the register is read successfully
                        eps_reg = (temp_reg_buff[0] << 8) + temp_reg_buff[1];

                        //Print out the register value
                        sprintf(print_buff[Interface], "OK+%04X", eps_reg);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    else{
                        //Reading has failed. Possibly the EPS is missing
                        sprintf(print_buff[Interface], "ERR - executing");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
                else
                {
                    // Wrong register number
                    sprintf(print_buff[Interface], "ERR - parameter");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }
            }
            else
            if (txline[Interface][4] == ANT_I2C_ADDRESS)
            {
                HAL_StatusTypeDef AntStat;
                uint8_t timeout = 0;
                uint8_t tmp[ANT_PAR_LEN];

                cmd_length = 8;
                if( len == cmd_length + ESTTC_CYMBOLS_IN_CRC )
                {
                    CRC_value_calc = crc32(0, (BYTE *)begin, len-ESTTC_CYMBOLS_IN_CRC);

                    CRC_value_rx = ESTTC_ExtractCRC(&begin[cmd_length+1]);

                    if( CRC_value_calc != CRC_value_rx )
                    {
                      sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      return ProcessedPacket;
                    }

                }else if( len != cmd_length )
                {
                    sprintf(print_buff[Interface], "ERR - length");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    return ProcessedPacket;
                }

                MX_I2C1_Init(); //Enable I2C1 interface
                do
                {

                    if( timeout >= 1 )
                    {
                        // if there is any error reset the I2C interface
                        if(( AntStat == HAL_ERROR )||( AntStat == HAL_TIMEOUT)||(( AntStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY)))
                        {
                            I2C_Reset(&hi2c1);
                        }
                        osDelay(5);

                        if( timeout >= 10 )
                        {
                            // Stop trying after certain times
                            break;
                        }
                    }

                    //Read the register from the EPS
                    AntStat = HAL_I2C_Master_Receive(&hi2c1, ANT_I2C_ADDRESS<<1, tmp, ANT_PAR_LEN, 10);


                    timeout ++; //count one more try
                }while( AntStat  != HAL_OK);
                HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface

                if ( HAL_OK == AntStat )
                {
                    //the registers are read successfully
                    ant_io[0] = tmp[0];
                    ant_io[1] = tmp[1];
                    ant_io[2] = tmp[2];
                    ant_io[3] = tmp[3];

                    /* Print the four registers with the status of the antenna */
                    sprintf(print_buff[Interface], "OK+%02X%02X%02X%02X", ant_io[0], ant_io[1], ant_io[2], ant_io[3]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }else{
                    //Reading has failed. Possibly the UHF antenna is missing
                    sprintf(print_buff[Interface], "ERR The UHF Antenna not connected");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }
            }
            else if (txline[Interface][4] == OBC_I2C_ADDRESS)
            {

              switch(txline[Interface][5])
              {
                case ESTTC_CMD_ACCSEL_1_DATA: //0x00:
                  if (AIS328DQ_GetAccAxesRaw(AIS328DQ_1_MEMS_I2C_ADDRESS, &data) == SEN_SUCCESS) { 

                    sprintf(print_buff[Interface], "OK+1%d/%d/%d", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    sprintf(print_buff[Interface], "Accelerometer 1 X=%d Y=%d Z=%d", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else {
                    I2C_Reset(&hi2c2);
                    sprintf(print_buff[Interface], "ERR - Accelerometer 1 fail - I2C bus restart is performed, please try command again");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  break;
                case ESTTC_CMD_ACCSEL_1_ACESS:	//0x01
                  if(SEN_SUCCESS == AIS328DQ_ReadReg(AIS328DQ_1_MEMS_I2C_ADDRESS, txline[Interface][6], (uint8_t *)&txline[Interface][7])) {
                    sprintf(print_buff[Interface], "OK+1%02X/%02X", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    sprintf(print_buff[Interface], "Accelerometer 1 register %X has value %X", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                    sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  break;
                  
                case ESTTC_CMD_ACCSEL_2_DATA: //0x02:
                  if (AIS328DQ_GetAccAxesRaw(AIS328DQ_2_MEMS_I2C_ADDRESS, &data) == SEN_SUCCESS) { 
                    sprintf(print_buff[Interface], "OK+2%d/%d/%d", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    sprintf(print_buff[Interface], "Accelerometer 2 X=%d Y=%d Z=%d", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else {
                    I2C_Reset(&hi2c2);
                    sprintf(print_buff[Interface], "Accelerometer 2 fail - I2C bus restart is performed, please try command again");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  } 
                  break;                  

                case ESTTC_CMD_ACCSEL_2_ACESS: //0x03:
                  if(SEN_SUCCESS == AIS328DQ_ReadReg(AIS328DQ_2_MEMS_I2C_ADDRESS, txline[Interface][6], (uint8_t *)&txline[Interface][7])) {
                    sprintf(print_buff[Interface], "OK+2%02X/%02X", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    sprintf(print_buff[Interface], "Accelerometer 2 register %X, has value %X", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                    sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  break;
                  
                case ESTTC_CMD_MAG1_DATA: //0x04:
                      if (Magnitometers_LIS3MDL_Read_Data(&B_raw, LIS3MDL_MAG_I2C_ADDRESS_LOW) != E_OK) {
                        I2C_Reset(&hi2c2);
                        sprintf(print_buff[Interface], "ERR - executing");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else {
                        sprintf(print_buff[Interface], "OK+1%1.0f/%01.0f/%1.0f", B_raw.AXIS_X, B_raw.AXIS_Y, B_raw.AXIS_Z);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Magnetometer 1 - Magnetic field in specific range X=%1.0f Y=%1.0f Z=%1.0f", B_raw.AXIS_X, B_raw.AXIS_Y, B_raw.AXIS_Z);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }  
                  break;                  

                case ESTTC_CMD_MGA1_ACESS: //0x05:
                  if (SEN_SUCCESS == LIS3MDL_MAG_ReadReg(LIS3MDL_MAG_I2C_ADDRESS_LOW, txline[Interface][6], (uint8_t *) &txline[Interface][7])) {
                    sprintf(print_buff[Interface], "OK+1%02X/%02X", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    sprintf(print_buff[Interface], "Magnetometer 1 reg No %d has value %d", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                    sprintf(print_buff[Interface], "ERR - executing");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  break;
                  
                case ESTTC_CMD_MAG2_DATA: //0x06:
                      Magnitometers_LIS3MDL_Init(LIS3MDL_MAG_I2C_ADDRESS_HIGH);
                      if (Magnitometers_LIS3MDL_Read_Data(&B_raw, LIS3MDL_MAG_I2C_ADDRESS_HIGH) != E_OK){
                          sprintf(print_buff[Interface], "ERR - executing");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else {
                        sprintf(print_buff[Interface], "OK+2%1.0f/%01.0f/%1.0f", B_raw.AXIS_X, B_raw.AXIS_Y, B_raw.AXIS_Z);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Magnetometer 2 -  Magnetic field in specific range X=%1.0f Y=%1.0f Z=%1.0f", B_raw.AXIS_X, B_raw.AXIS_Y, B_raw.AXIS_Z);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }                         
                  break;      
                  
                case ESTTC_CMD_MGA2_ACESS: //0x07:
                  if (SEN_SUCCESS == LIS3MDL_MAG_ReadReg(LIS3MDL_MAG_I2C_ADDRESS_HIGH, txline[Interface][6], (uint8_t *) &txline[Interface][7])) {
                    sprintf(print_buff[Interface], "OK+2%02X/%02X", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    sprintf(print_buff[Interface], "Magnetometer 2 reg No %d has value %d", txline[Interface][6], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                    sprintf(print_buff[Interface], "ERR - executing");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  break;                 

                case ESTTC_CMD_GYR1_X_RADIO_DATA: //0x08:
                  if ((ADIS16265_GetAxesRate(PANLE_GYROS_AXIS_X, &data) == SEN_SUCCESS) && (data.AXIS_X != 0x7FFF))   {
                     sprintf(print_buff[Interface], "OK+1%d", data.AXIS_X);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                     sprintf(print_buff[Interface], "Gyroscope 1 Data X=%5d", data.AXIS_X);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;

                case ESTTC_CMD_GYR1_X_ANGLE_DATA: //0x09:
                  if ((ADIS16265_GetAxesAngle(PANLE_GYROS_AXIS_X, &data) == SEN_SUCCESS) && (data.AXIS_X != 0x7FFF))   {
                     sprintf(print_buff[Interface], "OK+1%d", data.AXIS_X);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                     sprintf(print_buff[Interface], "Gyroscope 1 Angle X=%5d", data.AXIS_X);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;    
                
                case ESTTC_CMD_GYR1_X_AB_DATA: //0x0A:
                  if (ADIS16265_ReadReg16(txline[Interface][6], (uint16_t *)&utmp, PAN_X_M) == SEN_SUCCESS)   {
                      sprintf(print_buff[Interface], "OK+1%02X/%02X", txline[Interface][6], utmp);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Gyroscope 1 reg No %d has value %d", txline[Interface][6], utmp);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  } 
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;                               
                
                case ESTTC_CMD_GYR2_Y_RADIO_DATA: //0x0B:
                  if ((ADIS16265_GetAxesRate(PANLE_GYROS_AXIS_Y, &data) == SEN_SUCCESS) && (data.AXIS_Y != 0x7FFF))   {
                     sprintf(print_buff[Interface], "OK+2%d", data.AXIS_Y);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                     sprintf(print_buff[Interface], "Gyroscope 2 Data Y=%5d", data.AXIS_Y);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;

                case ESTTC_CMD_GYR2_Y_ANGLE_DATA: //0x0C:
                  if ((ADIS16265_GetAxesAngle(PANLE_GYROS_AXIS_Y, &data) == SEN_SUCCESS) && (data.AXIS_Y != 0x7FFF))   {
                     sprintf(print_buff[Interface], "OK+2%d", data.AXIS_Y);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                     sprintf(print_buff[Interface], "Gyroscope 2 Angle Y=%5d", data.AXIS_Y);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break; 
                
                case ESTTC_CMD_GYR2_Y_AB_DATA: //0x0D:
                  if (ADIS16265_ReadReg16(txline[Interface][6], (uint16_t *)&utmp, PAN_Y_M) == SEN_SUCCESS)   {
                      sprintf(print_buff[Interface], "OK+2%02X/%02X", txline[Interface][6], utmp);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Gyroscope 2 reg No %d has value %d", txline[Interface][6], utmp);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  } 
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;
                
                case ESTTC_CMD_GYR3_Z_RADIO_DATA: //0x0E:
                  if ((ADIS16265_GetAxesRate(PANLE_GYROS_AXIS_Z, &data) == SEN_SUCCESS) && (data.AXIS_Z != 0x7FFF))   {
                     sprintf(print_buff[Interface], "OK+3%d", data.AXIS_Z);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                     sprintf(print_buff[Interface], "Gyroscope 3 Data Z=%5d", data.AXIS_Z);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;

                case ESTTC_CMD_GYR3_Z_ANGLE_DATA: //0x0F:
                  if ((ADIS16265_GetAxesAngle(PANLE_GYROS_AXIS_Z, &data) == SEN_SUCCESS) && (data.AXIS_Z != 0x7FFF))  {
                     sprintf(print_buff[Interface], "OK+3%d", data.AXIS_Z);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                     sprintf(print_buff[Interface], "Gyroscope 3 Angle Z=%5d", data.AXIS_Z);
                     ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;   
                
                case ESTTC_CMD_GYR4_Z_AB_DATA: //0x10:
                  if (ADIS16265_ReadReg16(txline[Interface][6], (uint16_t *)&utmp, PAN_Z_M) == SEN_SUCCESS)  {
                      sprintf(print_buff[Interface], "OK+3%02X/%02X", txline[Interface][6], utmp);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Gyroscope 3 reg No %d has value %d", txline[Interface][6], utmp);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  } 
                  else{
                      sprintf(print_buff[Interface], "ERR - executing");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                break;                    
                  
                case ESTTC_CMD_TEMP_PANEL_X_P: //0x14:
                  Panels_Init();
                  if ((TMP122_GetTemperatureP(&tmperature_sens) == SEN_SUCCESS) && (PanelStat & (1<< PAN_X_P))) {
                      tp = ((tmperature_sens.Temp_X/8)*10)/16;
                      sprintf(print_buff[Interface], "OK+1%04X",(uint16_t)tmperature_sens.Temp_X);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Temperature Panel 1 (PAN1) =%3d.%d", tp/10, tp%10);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }else{
                      sprintf(print_buff[Interface], "ERR - Panel 1 is not attached!");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }
                   break;
                  
                case ESTTC_CMD_TEMP_PANEL_Y_P: //0x15:
                  Panels_Init();
                  if ((TMP122_GetTemperatureP(&tmperature_sens) == SEN_SUCCESS) && (PanelStat & (1<< PAN_Y_P))) {
                      tp = ((tmperature_sens.Temp_Y/8)*10)/16;
                      sprintf(print_buff[Interface], "OK+2%04X",(uint16_t)tmperature_sens.Temp_Y);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Temperature Panel 2 (PAN2) =%3d.%d", tp/10, tp%10);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   } else{
                      sprintf(print_buff[Interface], "ERR - Panel 2 is not attached!");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }
                   break;
                   
                case ESTTC_CMD_TEMP_PANEL_Z_P: //0x16:
                  Panels_Init();
                  if ((TMP122_GetTemperatureP(&tmperature_sens) == SEN_SUCCESS) && (PanelStat & (1<< PAN_Z_P))) {
                      tp = ((tmperature_sens.Temp_Z/8)*10)/16;
                      sprintf(print_buff[Interface], "OK+3%04X",(uint16_t)tmperature_sens.Temp_Z);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Temperature Panel 3 (PAN3) =%3d.%d", tp/10, tp%10);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   } else{
                       sprintf(print_buff[Interface], "ERR - Panel 3 is not attached!");
                       ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }
                   break;
                   
                case ESTTC_CMD_TEMP_PANEL_X_M: //0x17:
                  Panels_Init();
                  if ((TMP122_GetTemperatureM(&tmperature_sens) == SEN_SUCCESS) && (PanelStat & (1<< PAN_X_M))) {
                      tp = ((tmperature_sens.Temp_X/8)*10)/16;
                      sprintf(print_buff[Interface], "OK+4%04X",(uint16_t)tmperature_sens.Temp_X);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Temperature Panel 4 (PAN4) =%3d.%d", tp/10, tp%10);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   } else{
                      sprintf(print_buff[Interface], "ERR - Panel 4 is not attached!");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }
                   break;
                  
                case ESTTC_CMD_TEMP_PANEL_Y_M: //0x18:
                  Panels_Init();
                  if ((TMP122_GetTemperatureM(&tmperature_sens) == SEN_SUCCESS) && (PanelStat & (1<< PAN_Y_M))) {
                      tp = ((tmperature_sens.Temp_Y/8)*10)/16;
                      sprintf(print_buff[Interface], "OK+5%04X",(uint16_t)tmperature_sens.Temp_Y);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Temperature Panel 5 (PAN5) =%3d.%d", tp/10, tp%10);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   } else{
                       sprintf(print_buff[Interface], "ERR - Panel 5 is not attached!");
                       ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }
                   break;
                   
                case ESTTC_CMD_TEMP_PANEL_Z_M: //0x19:
                  Panels_Init();
                  if ((TMP122_GetTemperatureM(&tmperature_sens) == SEN_SUCCESS) && (PanelStat & (1<< PAN_Z_M))) {
                      tp = ((tmperature_sens.Temp_Z/8)*10)/16;
                      sprintf(print_buff[Interface], "OK+6%04X",(uint16_t)tmperature_sens.Temp_Z);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      sprintf(print_buff[Interface], "Temperature Panel 6 (PAN6) =%3d.%d", tp/10, tp%10);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   } else{
                      sprintf(print_buff[Interface], "ERR - Panel 6 is not attached!");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   }
                   break;                                     

                case ESTTC_CMD_PHOTO_PANEL_1: //0x1A:
                   Panel_GetPhotodiodesLum();

                   sprintf(print_buff[Interface], "OK+1%02X", PanelLight[0]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                   sprintf(print_buff[Interface], "Panel Light 1 (PAN1) =%4u", PanelLight[0]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   break;
                   
                case ESTTC_CMD_PHOTO_PANEL_2: //0x1B:
                   Panel_GetPhotodiodesLum();

                   sprintf(print_buff[Interface], "OK+2%02X", PanelLight[1]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                   sprintf(print_buff[Interface], "Panel Light 2 (PAN2) =%4u", PanelLight[1]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   break;
                   
                case ESTTC_CMD_PHOTO_PANEL_3: //0x1C:
                   Panel_GetPhotodiodesLum();
                   sprintf(print_buff[Interface], "OK+3%02X", PanelLight[2]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                   sprintf(print_buff[Interface], "Panel Light 3 (PAN3) =%4u", PanelLight[2]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   break;
                                     
                case ESTTC_CMD_PHOTO_PANEL_4: //0x1D:
                   Panel_GetPhotodiodesLum();
                   sprintf(print_buff[Interface], "OK+4%02X", PanelLight[3]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                   sprintf(print_buff[Interface], "Panel Light 4 (PAN4) =%4u", PanelLight[3]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   break;
                                              
                case ESTTC_CMD_PHOTO_PANEL_5: //0x1E:
                   Panel_GetPhotodiodesLum();
                   sprintf(print_buff[Interface], "OK+5%02X", PanelLight[4]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                   sprintf(print_buff[Interface], "Panel Light 5 (PAN5) =%4u", PanelLight[4]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   break;
                                              
                case ESTTC_CMD_PHOTO_PANEL_6: //0x1F:
                   Panel_GetPhotodiodesLum();
                   sprintf(print_buff[Interface], "OK+6%02X", PanelLight[5]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                   sprintf(print_buff[Interface], "Panel Light 6 (PAN6) =%4u", PanelLight[5]);
                   ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                   break;
                   
                case ESTTC_CMD_OUTPUT_CONTROL:
                    s8_tmp  =  HAL_GPIO_ReadPin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin);
                    s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT2_GPIO_Port, OBC_OUT2_Pin) << 1;
                    s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT3_GPIO_Port, OBC_OUT3_Pin) << 2;
                    s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin) << 3;
                    s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT6_GPIO_Port, OBC_OUT6_Pin) << 4;

                    sprintf(print_buff[Interface], "Output States 0x%X", s8_tmp);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    break;

                case ESTTC_CMD_GET_TIME: //0x31:   // get time

                  HAL_RTC_WaitForSynchro(&hrtc);
                  HAL_RTC_GetTime(&hrtc, &sTime, calendar_format); /* first read just copy from data to shadow register. The second read, gets the real value from the shadow registers */
                  HAL_RTC_GetDate(&hrtc, &sDate, calendar_format);
                  osDelay(1);/* wait time to copy the data to shadow registers */

                  if (HAL_OK == HAL_RTC_GetTime(&hrtc, &sTime, calendar_format))
                  {
                    sprintf(print_buff[Interface], "OK TIME %02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }else{
                    sprintf(print_buff[Interface], "ERR - executing");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                    HAL_RTC_GetDate(&hrtc, &sDate, calendar_format); // synchronization RTC date/time
                  break;

                case ESTTC_CMD_GET_DATA: //0x33:   // get date

                  HAL_RTC_WaitForSynchro(&hrtc);
                  HAL_RTC_GetTime(&hrtc, &sTime, calendar_format); /* first read just copy from data to shadow register. The second read, gets the real value from the shadow registers */
                  HAL_RTC_GetDate(&hrtc, &sDate, calendar_format);
                  osDelay(1);/* wait time to copy the data to shadow registers */

                  if (HAL_OK == HAL_RTC_GetDate(&hrtc, &sDate, calendar_format)){
                    sprintf(print_buff[Interface], "OK DATE YY/MM/DD %02d / %02d / %02d", sDate.Year, sDate.Month, sDate.Date);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  else{
                    sprintf(print_buff[Interface], "ERR - executing");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  }
                  break;

                case ESTTC_CMD_UPTIME: //0x35    // get uptime
                {
                    sprintf(print_buff[Interface], "OK UPTIME DDDDD:HH:MM:SS %05d:%02d:%02d:%02d", (int)up_day, up_hrs, up_min, up_sec);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }break;

#ifdef ENABLE_OBC_ADCS
                case ESTTC_CMD_GET_ADCS_STATUS: // 0x42
                {
                    char algoNames[3][16];

                    if (ADCS_ERR == ADCS_GetStatus ((ADCS_Status_t *)ADCSStatus)) {
                        sprintf (print_buff[Interface], "OK ADCS Status: *** NOT LICENSED *** for System UUID %08X-%08X-%08X",
                                (unsigned int)(STM32_UUID[0]), (unsigned int)(STM32_UUID[1]), (unsigned int)(STM32_UUID[2]));
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        break;
                    }
                    // Detumbling algo
                    switch (ADCSStatus[0].algorithm[0]) {
                        case ADCS_ALGO_BDOT_ES:
                            sprintf (algoNames[0], "BDOT_PD_ES");
                            break;
                        case ADCS_ALGO_BDOT_ADE:
                            sprintf (algoNames[0], "BDOT_ADE");
                            break;
                        case ADCS_ALGO_DTORQLINEAR:
                            sprintf (algoNames[0], "BDOT_DTORQ_LINE");
                            break;
                        case ADCS_ALGO_DTORQCUBIC:
                            sprintf (algoNames[0], "BDOT_DTORQ_CUBE");
                            break;
                        default:
                            sprintf (algoNames[0], "UNKNOWN");
                            break;
                    }
                    // Determination algo
                    switch (ADCSStatus[1].algorithm[0]) {
                        case ADCS_ALGO_TRIAD_ES:
                            sprintf (algoNames[1], "TRIAD");
                            break;
                        default:
                            sprintf (algoNames[1], "UNKNOWN");
                            break;
                    }
                    // Control algo
                    switch (ADCSStatus[1].algorithm[1]) {
                        case ADCS_ALGO_CTRL_NONE:
                            sprintf (algoNames[2], "NONE");
                            break;
                        case ADCS_ALGO_CTRL_PID_ES:
                            sprintf (algoNames[2], "PID_ES");
                            break;
                        default:
                            sprintf (algoNames[2], "UNKNOWN");
                            break;
                    }
                    sprintf (print_buff[Interface], "OK ADCS Status: LICENSED for System UUID %08X-%08X-%08X",
                            (unsigned int)(STM32_UUID[0]), (unsigned int)(STM32_UUID[1]), (unsigned int)(STM32_UUID[2]));
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    sprintf (print_buff[Interface], "<<< DETUMBLING >>> %s    status: %s    algorithm: %s    magnetometer errors: %u    gyro errors: %u    gyros: %s    stability: %d    state: %u    quarantine: %s",
                            (ADCSStatus[0].flags & ADCS_STATUS_LOWBAT) ? "!!! LOW BATTERY !!!" : "",
                            ADCSStatus[0].systemEnabled ? "ACTIVE" : "INACTIVE", algoNames[0],
                            (unsigned int)ADCSStatus[0].magErrors, (unsigned int)ADCSStatus[0].gyroErrors,
                            ADCSStatus[0].gyrosEnabled ? "Enabled" : "Disabled", (int)ADCSStatus[0].stability,
                            ADCSStatus[0].state, ADCSStatus[0].qTick ? "ACTIVE" : "INACTIVE");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    sprintf (print_buff[Interface], "<<< DETERMNINATION & CONTROL >>> %s    status: %s    determination algorithm: %s    control algorithm: %s    magnetometer errors: %u    gyro errors: %u    gyros: %s    stability: %d    state: %u    mode: %s",
                            (ADCSStatus[1].flags & ADCS_STATUS_LOWBAT) ? "!!! LOW BATTERY !!!" : "",
                            ADCSStatus[1].systemEnabled ? "ACTIVE" : "INACTIVE", algoNames[1], algoNames[2],
                            (unsigned int)ADCSStatus[1].magErrors, (unsigned int)ADCSStatus[1].gyroErrors,
                            ADCSStatus[1].gyrosEnabled ? "Enabled" : "Disabled", (int)ADCSStatus[1].stability,
                            ADCSStatus[1].state,
                            (ADCSStatus[1].flags & ADCS_STATUS_TGTEN) ? "TARGETING" : "AUTOMATIC");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    break;
                }
#endif /* ENABLE_OBC_ADCS */

                case ESTTC_CMD_RST_COUNTS:  // 0x61
                {
                    sprintf(print_buff[Interface], "RST Counters: 0.WWD = %03d / 1.IWD = %03d / 2.LPR = %03d / 3.POR = %03d / 4.RstPin = %03d / 5.BOR = %03d / 6.HardFault = %03d / 7.MemFault = %03d / 8.BusFault = %03d / 9.UsageFault = %03d",
                    (int)BootData->RST_WWD, (int)BootData->RST_IWD, (int)BootData->RST_LPR, (int)BootData->RST_POR, (int)BootData->RST_RstPin, (int)BootData->RST_BOR, (int)BootData->RST_HardFault,
                    (int)BootData->RST_MemFault, (int)BootData->RST_BusFault, (int)BootData->RST_UsageFault);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }break;

                case ESTTC_CMD_RESET:   // get s/w version
                          sprintf(print_buff[Interface], "OK+APPL v.%d.%0d / <%s %s>",
                    	          verFW_MAJOR_REV_NB, verFW_MINOR_REV_NB,
                    				__DATE__, __TIME__);
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  break;

                default:
                    sprintf(print_buff[Interface], "ERR - parameter");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                  break;
              }
            }
            else{
              sprintf(print_buff[Interface], "ERR addr");
              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
            }
          }
          else
          if (txline[Interface][3] == 'W')
          {
            if (txline[Interface][4] == S_X_BAND_ADDRESS)
            {
                    uint8_t  data_size;
                    uint16_t Identifier = (txline[Interface][7]<<8) + txline[Interface][8];
                    SX_BAND_Selection_Enum SX_BAND_Select_Module;

                    if((Identifier >= S_BAND_ID_RANGE_MIN)&&(Identifier <= S_BAND_ID_RANGE_MAX))
                    {
                        data_size  = txline[Interface][6] - S_BAND_ID_LENGTH;
                        SX_BAND_Select_Module = SX_SELECT_S_BAND_TRANSCEIVER;

                        sprintf(print_buff[Interface], "Requesting S-Band CMD 0x%X ... ",txline[Interface][5]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }else if((Identifier >= X_BAND_ID_RANGE_MIN)&&(Identifier <= X_BAND_ID_RANGE_MAX))
                    {
                        data_size  = txline[Interface][6] - X_BAND_ID_LENGT;
                        SX_BAND_Select_Module = SX_SELECT_X_BAND_TRANSCEIVER;

                        sprintf(print_buff[Interface], "Requesting X-Band CMD 0x%X ... ",txline[Interface][5]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }else{
                        sprintf(print_buff[Interface], "Err - Unknown ID %X", Identifier);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }

                    switch( txline[Interface][5] )
                    {
                        case 0x01:
                        case 0x02:
                        case 0x04:
                        case 0x05:
                        case 0x06:
                        case 0x07:
                        case 0x0C:
                        {
                            for(uint8_t i = 0; i < 2 ; i++)
                            {
                                txline[Interface][9+i] = begin[14+i] - '0';
                            }

                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }
                        }break;

                        case 0x30:
                        case 0x31:
                        case 0x32:
                        {
                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', 0, NULL);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', 0, NULL);
                            }
                        }break;

#ifdef ENABLE_SX_BAND_TESTBOARD
                        case 0x0D:
                        {
                            for(uint8_t i = 0; i < 168 ; i++)
                            {
                                txline[Interface][9+i] = begin[14+i] - '0';
                            }

                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }
                        }break;
#endif /* #ifdef ENABLE_SX_BAND_TESTBOARD */

                        case 0x03:
                        {
                            for(uint8_t i = 0; i < 8 ; i++)
                            {
                                txline[Interface][9+i] = begin[14+i] - '0';
                            }

                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }
                        }break;

                        case 0x08:
                        {
                            for(uint8_t i = 0; i < 4 ; i++)
                            {
                                txline[Interface][9+i] = begin[14+i] - '0';
                            }

                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }
                        }break;

                        case 0x09:
                        {
                            for(uint8_t i = 0; i < 24 ; i++)
                            {
                                txline[Interface][9+i] = begin[14+i] - '0';
                            }

                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }
                        }break;

#ifdef ENABLE_SX_BAND_TESTBOARD
                        case 0x81: // COMMAND_PD_CLEAR_WARNINGS - expecting no input
                        case 0x82: // COMMAND_PD_CALIBRATION_A - expecting no input
                        case 0x83: // COMMAND_PD_REWRITE_CONFIG - expecting no input
                        case 0x84: // COMMAND_PD_RESET - expecting no input
                            if (SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER) {
                                S_BAND_TRNSM_ESTTC_StartCmd (Identifier, txline[Interface][5], 'W', 0, NULL);
                            } else {
                                X_BAND_TRNSM_ESTTC_StartCmd (Identifier, txline[Interface][5], 'W', 0, NULL);
                            }
                            break;
                        case 0x80: // COMMAND_PD_RF_OUT_ENB_DIS - expecting U16 input - 0 or 1
                        case 0x85: // COMMAND_PD_ADAPTATION_ENB_DIS - expecting U16 input - 0 or 1
                            for (uint8_t i = 0; i < 2 ; i++) {
                                txline[Interface][9 + i] = begin[14 + i] - '0';
                            }
                            if (SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER) {
                                S_BAND_TRNSM_ESTTC_StartCmd (Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            } else {
                                X_BAND_TRNSM_ESTTC_StartCmd (Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&txline[Interface][9]);
                            }
                            break;
#endif /* ENABLE_SX_BAND_TESTBOARD */

                        default:
                        {
                            sprintf(print_buff[Interface], "Err - Invalid CMD");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
            }else if (txline[Interface][4] == EPS_I2C_ADDRESS)
            {
                uint8_t timeout = 0;
                HAL_StatusTypeDef I2C_retStat;

                cmd_length = 14;
                if( len == cmd_length + ESTTC_CYMBOLS_IN_CRC )
                {
                    CRC_value_calc = crc32(0, (BYTE *)begin, len-ESTTC_CYMBOLS_IN_CRC);

                    CRC_value_rx = ESTTC_ExtractCRC(&begin[cmd_length+1]);

                    if( CRC_value_calc != CRC_value_rx )
                    {
                      sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      return ProcessedPacket;
                    }

                }else if( len != cmd_length )
                {
                    sprintf(print_buff[Interface], "ERR - length");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    return ProcessedPacket;
                }

                if (txline[Interface][7] < EPS_OUT_PAR_NUM)
                {
                    MX_I2C1_Init(); //Enable I2C1 interface
                    do
                    {

                        if( timeout >= 1 )
                        {
                            if(( I2C_retStat == HAL_ERROR )||( I2C_retStat == HAL_TIMEOUT)||(( I2C_retStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY)))
                            {
                                I2C_Reset(&hi2c1);
                            }
                            osDelay(5);

                            if( timeout >= 10 )
                            {
                                break;
                            }
                        }
                        I2C_retStat = HAL_I2C_Master_Transmit(&hi2c1, EPS_I2C_ADDRESS<<1, (uint8_t *)&txline[Interface][7], 2, 10);

                        timeout ++;
                    }while(I2C_retStat != HAL_OK);
                    HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface

                    if (HAL_OK == I2C_retStat)
                    {
                        sprintf(print_buff[Interface], "OK EPS CMD %X, Reg %x", txline[Interface][5], txline[Interface][7]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    else{
                        sprintf(print_buff[Interface], "ERR - executing");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
                else
                {
                    sprintf(print_buff[Interface], "ERR - parameter");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }

            }
            else
            if (txline[Interface][4] == ANT_I2C_ADDRESS)
            {
                uint8_t timeout = 0;
                HAL_StatusTypeDef I2C_retStat;

                cmd_length = 12;
                if( len == cmd_length + ESTTC_CYMBOLS_IN_CRC )
                {
                    CRC_value_calc = crc32(0, (BYTE *)begin, len-ESTTC_CYMBOLS_IN_CRC);

                    CRC_value_rx = ESTTC_ExtractCRC(&begin[cmd_length+1]);

                    if( CRC_value_calc != CRC_value_rx )
                    {
                      sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      return ProcessedPacket;
                    }

                }else if( len != cmd_length )
                {
                    sprintf(print_buff[Interface], "ERR - length");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    return ProcessedPacket;
                }

                /* Write to Communication interface I2C */
                MX_I2C1_Init(); //Enable I2C1 interface
                do
                {
                    if( timeout >= 1 )
                    {
                        if(( I2C_retStat == HAL_ERROR )||( I2C_retStat == HAL_TIMEOUT)||(( I2C_retStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY)))
                        {
                            I2C_Reset(&hi2c1);
                        }
                        osDelay(5);

                        if( timeout >= 10 )
                        {
                            break;
                        }
                    }
                    I2C_retStat = HAL_I2C_Master_Transmit(&hi2c1, ANT_I2C_ADDRESS<<1, (uint8_t *)&txline[Interface][7], 1, 10);

                    timeout ++;
                }while(I2C_retStat != HAL_OK);
                HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface

                if (HAL_OK == I2C_retStat)
                {
                    sprintf(print_buff[Interface], "OK ANT CMD %X, Val 0x%x", txline[Interface][5], txline[Interface][7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }
                else{
                    sprintf(print_buff[Interface], "ERR - executing");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }
            }
            else
            if (txline[Interface][4] == OBC_I2C_ADDRESS)
            {
              if (txline[Interface][5] <= ESTTC_CMD_PAR_NUM)
              {
                  switch(txline[Interface][5])
                  {
                    case ESTTC_CMD_ACCSEL_1_ACESS: // 0x01
                      if(SEN_SUCCESS == AIS328DQ_WriteReg(AIS328DQ_1_MEMS_I2C_ADDRESS,txline[Interface][7], txline[Interface][8])) {
                        sprintf(print_buff[Interface], "OK+1%02X/%02X", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Accelerometer 1 register %X is set with value %X", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                        sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;

                    case ESTTC_CMD_ACCSEL_2_ACESS: //0x03:
                      if(SEN_SUCCESS == AIS328DQ_WriteReg(AIS328DQ_2_MEMS_I2C_ADDRESS,txline[Interface][7], txline[Interface][8])) {
                        sprintf(print_buff[Interface], "OK+2%02X/%02X", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Accelerometer 2 register %X is set with value %X", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                        sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;

                    case ESTTC_CMD_MGA1_ACESS: //0x05:
                      if (SEN_SUCCESS == LIS3MDL_MAG_WriteReg(LIS3MDL_MAG_I2C_ADDRESS_LOW, txline[Interface][7], txline[Interface][8])) {
                        sprintf(print_buff[Interface], "OK+1%02X/%02X", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Magnetometer 1 set reg No %d with value %d", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                          sprintf(print_buff[Interface], "ERR - executing");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;

                    case ESTTC_CMD_MGA2_ACESS: //0x07:
                      if (SEN_SUCCESS == LIS3MDL_MAG_WriteReg(LIS3MDL_MAG_I2C_ADDRESS_HIGH, txline[Interface][7], txline[Interface][8])) {
                        sprintf(print_buff[Interface], "OK+2%02X/%02X", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Magnetometer 2 set reg No %d with value %d", txline[Interface][7], txline[Interface][8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                        sprintf(print_buff[Interface], "ERR - executing");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;                     
                      
                    case ESTTC_CMD_GYR1_X_AB_DATA: //0x0A:
                      utmp = ((uint16_t)txline[Interface][8] << 8) + (uint16_t)txline[Interface][9];
                      if (SEN_SUCCESS == ADIS16265_WriteReg16(txline[Interface][7], utmp, PAN_X_M)) {
                        sprintf(print_buff[Interface], "OK+1%02X/%02X", txline[Interface][7], utmp);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Gyroscope 1 set reg No %d with value %d", txline[Interface][7], utmp);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                          sprintf(print_buff[Interface], "ERR - executing");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;   
                      
                    case ESTTC_CMD_GYR2_Y_AB_DATA: //0x0D:
                      utmp = ((uint16_t)txline[Interface][8] << 8) + (uint16_t)txline[Interface][9];
                      if (SEN_SUCCESS == ADIS16265_WriteReg16(txline[Interface][7], utmp, PAN_Y_M)) {
                        sprintf(print_buff[Interface], "OK+2%02X/%02X", txline[Interface][7], utmp);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Gyroscope 2 set reg No %d with value %d", txline[Interface][7], utmp);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                        sprintf(print_buff[Interface], "ERR - executing");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;                                          

                    case ESTTC_CMD_GYR4_Z_AB_DATA: //0x10:
                      utmp = ((uint16_t)txline[Interface][8] << 8) + (uint16_t)txline[Interface][9];
                      if (SEN_SUCCESS == ADIS16265_WriteReg16(txline[Interface][7], utmp, PAN_Z_M)) {
                        sprintf(print_buff[Interface], "OK+3%02X/%02X", txline[Interface][7], utmp);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        sprintf(print_buff[Interface], "Gyroscope 3 set reg No %d with value %d", txline[Interface][7], utmp);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                        sprintf(print_buff[Interface], "ERR - executing");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;                        

                    case ESTTC_CMD_MAGTRK1_POWER: //0x11:
                      if (txline[Interface][7] > 100) txline[Interface][8] = 100; // 100% is maximum
                      if (txline[Interface][8] > 1)  txline[Interface][9] = 1; // 0 - negative, 1 is positive direction
                      if (SetMagnetorque(PAN_X_M, txline[Interface][7], txline[Interface][8]) == SEN_SUCCESS) {
                          sprintf(print_buff[Interface], "OK+1%02X/X", txline[Interface][7]);
                          if(txline[Interface][8] == 0)
						  {
						    strcat(print_buff[Interface], "-");
						  }
						  else
						  {
						    strcat(print_buff[Interface], "+");
						  }
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                          sprintf(print_buff[Interface], "Set Magnetorquer 1 (TRQ1) on PAN 4 with power %d and direction X", txline[Interface][7]);
                          if(txline[Interface][8] == 0)
                          {
                            strcat(print_buff[Interface], "-");
                          }
                          else
                          {
                            strcat(print_buff[Interface], "+");
                          }
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                          sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;
                  
                    case ESTTC_CMD_MAGTRK2_POWER: //0x12:
                      if (txline[Interface][7] > 100) txline[Interface][8] = 100; // 100% is maximum
                      if (txline[Interface][8] > 1)  txline[Interface][9] = 1; // 0 - negative, 1 is positive direction
                      if (SetMagnetorque(PAN_Y_M, txline[Interface][7], txline[Interface][8]) == SEN_SUCCESS) {
                          sprintf(print_buff[Interface], "OK+2%02X/Y", txline[Interface][7]);
                          if(txline[Interface][8] == 0)
                            {
                              strcat(print_buff[Interface], "-");
                            }
                            else
                            {
                              strcat(print_buff[Interface], "+");
                            }
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                          sprintf(print_buff[Interface], "Set Magnetorquer 2 (TRQ2) on PAN 5 with power %d and direction Y", txline[Interface][7]);
                          if(txline[Interface][8] == 0)
                            {
                              strcat(print_buff[Interface], "-");
                            }
                            else
                            {
                              strcat(print_buff[Interface], "+");
                            }
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                          sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;
                      
                    case ESTTC_CMD_MAGTRK3_POWER: //0x13:
                      if (txline[Interface][7] > 100) txline[Interface][8] = 100; // 100% is maximum
                      if (txline[Interface][8] > 1)  txline[Interface][9] = 1; // 0 - negative, 1 is positive direction
                      if (SetMagnetorque(PAN_Z_M, txline[Interface][7], txline[Interface][8]) == SEN_SUCCESS) {
                          sprintf(print_buff[Interface], "OK+3%02X/Z", txline[Interface][7]);
                          if(txline[Interface][8] == 0)
                          {
                            strcat(print_buff[Interface], "-");
                          }
                          else
                          {
                            strcat(print_buff[Interface], "+");
                          }
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                          sprintf(print_buff[Interface], "Set Magnetorquer 3 (TRQ3) on PAN 6 with power %d and direction Z", txline[Interface][7]);
                          if(txline[Interface][8] == 0)
                          {
                            strcat(print_buff[Interface], "-");
                          }
                          else
                          {
                            strcat(print_buff[Interface], "+");
                          }
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      else{
                          sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      break;

                    case ESTTC_CMD_OUTPUT_CONTROL:

                        if (txline[Interface][8] < 2)
                        {
                            switch (txline[Interface][7])
                            {
                                case 1:
                                {
                                    HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, txline[Interface][8]);
                                }break;

                                case 2:
                                {
                                    HAL_GPIO_WritePin(OBC_OUT2_GPIO_Port, OBC_OUT2_Pin, txline[Interface][8]);
                                }break;

                                case 3:
                                {
                                    HAL_GPIO_WritePin(OBC_OUT3_GPIO_Port, OBC_OUT3_Pin, txline[Interface][8]);
                                }break;

                                case 5:
                                {
                                    HAL_GPIO_WritePin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin, txline[Interface][8]);
                                }break;

                                case 6:
                                {
                                    HAL_GPIO_WritePin(OBC_OUT6_GPIO_Port, OBC_OUT6_Pin, txline[Interface][8]);
                                }break;

                                default:
                                {
                                    sprintf(print_buff[Interface], "ERR - Invalid output!");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                            }

                            s8_tmp  =  HAL_GPIO_ReadPin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin);
                            s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT2_GPIO_Port, OBC_OUT2_Pin) << 1;
                            s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT3_GPIO_Port, OBC_OUT3_Pin) << 2;
                            s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin) << 3;
                            s8_tmp |=  HAL_GPIO_ReadPin(OBC_OUT6_GPIO_Port, OBC_OUT6_Pin) << 4;

                            sprintf(print_buff[Interface], "Output States 0x%X", s8_tmp);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }else{
                            sprintf(print_buff[Interface], "ERR - Invalid state!");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }

                        break;

                    case ESTTC_CMD_SET_TIME: //0x32:   // set time
                    {
                        uint8_t TempHours, TempMinutes, TempSeconds;

                        TempHours   = (uint16_t)((txline[Interface][7]-0x30)*10 + (txline[Interface][8]-0x30));
                        TempMinutes = (uint16_t)((txline[Interface][9]-0x30)*10 + (txline[Interface][10]-0x30));
                        TempSeconds = (uint16_t)((txline[Interface][11]-0x30)*10 + (txline[Interface][12]-0x30));

                        if( (TempHours < 24 ) && (TempMinutes < 60 ) && (TempSeconds < 60 ) )
                        {
                            sTime.Hours   = TempHours;
                            sTime.Minutes = TempMinutes;
                            sTime.Seconds = TempSeconds;

                            if (HAL_OK == HAL_RTC_SetTime(&hrtc, &sTime, calendar_format))
                            {
                              sprintf(print_buff[Interface], "OK TIME %02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            else{
                                sprintf(print_buff[Interface], "ERR - executing");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                        }else{
                            sprintf(print_buff[Interface], "ERR - Wrong parameters");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }
                    }
                    break;

                    case ESTTC_CMD_SET_DATA: //0x34:   // set date
                    {
                        uint8_t TempYear, TempMonth, TempDay;

                        TempYear  = (uint16_t)((txline[Interface][7]-0x30)*10 + (txline[Interface][8]-0x30));
                        TempMonth = (uint16_t)((txline[Interface][9]-0x30)*10 + (txline[Interface][10]-0x30));
                        TempDay   = (uint16_t)((txline[Interface][11]-0x30)*10 + (txline[Interface][12]-0x30));

                        if ( ESTTC_CheckDate(TempYear, TempMonth, TempDay) )
                        {
                            sDate.Year = TempYear;
                            sDate.Month = TempMonth;
                            sDate.Date = TempDay;

                            if (HAL_OK == HAL_RTC_SetDate(&hrtc, &sDate, calendar_format))
                            {
                              sprintf(print_buff[Interface], "OK DATE YY/MM/DD %02d / %02d / %02d", sDate.Year, sDate.Month, sDate.Date);
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            else{
                              sprintf(print_buff[Interface], "ERR - executing");
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                        }else{
                            sprintf(print_buff[Interface], "ERR - Wrong parameters");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }
                    }
                    break;

                    case ESTTC_CMD_UPTIME: //0x35    // set uptime
                    {
#ifdef ESTTC_FAULT_TESTS_ENABLE
                        uint32_t tmp_up_day=0;
                        uint8_t  tmp_up_sec=0, tmp_up_min=0, tmp_up_hrs=0;

                        tmp_up_day = (uint16_t)((txline[Interface][7]-0x30)*10 + (txline[Interface][8]-0x30));
                        tmp_up_hrs = (uint16_t)((txline[Interface][9]-0x30)*10 + (txline[Interface][10]-0x30));
                        tmp_up_min = (uint16_t)((txline[Interface][11]-0x30)*10 + (txline[Interface][12]-0x30));
                        tmp_up_sec = (uint16_t)((txline[Interface][13]-0x30)*10 + (txline[Interface][14]-0x30));

                        if(( tmp_up_hrs < 24 )&&( tmp_up_min < 60 )&&( tmp_up_sec < 60 ))
                        {
                            up_day = tmp_up_day;
                            up_hrs = tmp_up_hrs;
                            up_min = tmp_up_min;
                            up_sec = tmp_up_sec;

                            sprintf(print_buff[Interface], "OK UPTIME DDDDD:HH:MM:SS %05d:%02d:%02d:%02d", (int)up_day, up_hrs, up_min, up_sec);
                        }else{
                            sprintf(print_buff[Interface], "ERR UPTIME - Wrong params");
                        }

                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
#else   /* ESTTC_FAULT_TESTS_ENABLE */
                        sprintf(print_buff[Interface], "The command is disabled");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
#endif   /* ESTTC_FAULT_TESTS_ENABLE */
                    }break;

#ifdef ENABLE_OBC_ADCS
                    case ESTTC_CMD_SET_ADCS_TARGET: // 0x37
                        // ADCS Targeting - ECF = 1 / ECI = 0 + targeting wall X = 0 / Y = 1 / Z = 2 + 3x1 targeting vector (scalar is of type double)
                        ADCSTarget.targetPosIsECF = txline[Interface][7];
                        ADCSTarget.targetingWall = txline[Interface][8];
                        memcpy (&(ADCSTarget.targetPosECFECI.X.Axis), &txline[Interface][9], sizeof (REAL));
                        memcpy (&(ADCSTarget.targetPosECFECI.Y.Axis), &txline[Interface][9 + sizeof (REAL)], sizeof (REAL));
                        memcpy (&(ADCSTarget.targetPosECFECI.Z.Axis), &txline[Interface][9 + 2 * sizeof (REAL)], sizeof (REAL));
                        if (ADCS_OK == ADCS_SetState (ADCS_STATE_TARGETING_POS, (void *)&ADCSTarget, 0)) {
                            sprintf (print_buff[Interface], "OK ADCS TARGETING %s ref. frame, targeting wall %c, <%16.4f,%16.4f,%16.4f>",
                                    (ADCSTarget.targetPosIsECF == 0 ? "ECI" : "ECF"),
                                    (ADCSTarget.targetingWall == ADCS_TGTWALL_X ? 'X' : (ADCSTarget.targetingWall == ADCS_TGTWALL_Y ? 'Y' : 'Z')),
                                    ADCSTarget.targetPosECFECI.X.Axis, ADCSTarget.targetPosECFECI.Y.Axis, ADCSTarget.targetPosECFECI.Z.Axis);
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        } else {
                            sprintf (print_buff[Interface], "ERR - executing");
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        }
                        break;
                    case ESTTC_CMD_SET_ADCS: // 0x38
                        if (ADCS_OK == ADCS_SetState (ADCS_STATE_ADCS, NULL, 0)) {
                            sprintf (print_buff[Interface], "OK ADCS TARGETING set on AUTO");
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        } else {
                            sprintf (print_buff[Interface], "ERR - executing");
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        }
                        break;
                    case ESTTC_CMD_SET_ADCS_SETTINGS: // 0x39
                        if ((uint8_t)txline[Interface][7] > ADCS_ALGO_DTORQCUBIC ||
                                (uint8_t)txline[Interface][8] > ADCS_ALGO_TRIAD_ES ||
                                (uint8_t)txline[Interface][9] > ADCS_ALGO_CTRL_PID_ES) {
                            sprintf (print_buff[Interface], "ERR - Wrong ADCS Setup");
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        } else {
                            //memcpy (&EEPROM_emul_DataTemp.ADCSConfig, &txline[Interface][7], sizeof (EEPROM_emul_DataTemp.ADCSConfig));
                            EEPROM_emul_DataTemp.ADCSConfig.detumblingAlgo = (uint8_t)txline[Interface][7];
                            EEPROM_emul_DataTemp.ADCSConfig.determinationAlgo = (uint8_t)txline[Interface][8];
                            EEPROM_emul_DataTemp.ADCSConfig.controlAlgo = (uint8_t)txline[Interface][9];
                            memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig, &txline[Interface][10], sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig));
                            memcpy (&EEPROM_emul_DataTemp.ADCSConfig.magnetoGain, (REAL *)&txline[Interface][10 + sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig)], sizeof (REAL));
                            memcpy (&EEPROM_emul_DataTemp.ADCSConfig.gyroGain, (REAL *)&txline[Interface][10 +  sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig) + sizeof (REAL)], sizeof (REAL));
                            memcpy (&EEPROM_emul_DataTemp.ADCSConfig.allowedAngularDiff, (REAL *)&txline[Interface][10 +  sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig)+ 2 * sizeof (REAL)], sizeof (REAL));
                            memcpy (&EEPROM_emul_DataTemp.ADCSConfig.startDetumblingAt, (uint64_t *)&txline[Interface][10 +  sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig) + 3 * sizeof (REAL)], sizeof (uint64_t));
                            EEPROM_emul_DataTemp.ADCSConfig.configMask = txline[Interface][10 +  sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.photoSensorConfig) + 3 * sizeof (REAL) + sizeof (uint64_t)];
                            /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
                            EEPROM_Emul_SyncInfo();
                            /* Inform the user */
                            sprintf (print_buff[Interface], "OK ADCS Setup complete");
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        }
                        break;
                    case ESTTC_CMD_SET_ADCS_DETERM_SETUP: // 0x40
                        //memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig, &txline[Interface][7], sizeof (EEPROM_emul_DataTemp.ADCSConfig.determinationConfig));
                        memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.iniPosition, (Vec3D_t *)&txline[Interface][7], sizeof (Vec3D_t));
                        memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.iniVelocity, (Vec3D_t *)&txline[Interface][7 + sizeof (Vec3D_t)], sizeof (Vec3D_t));
                        memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.satMass, (REAL *)&txline[Interface][7 + 2 * sizeof (Vec3D_t)], sizeof (REAL));
                        memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.satDragCoeff, (REAL *)&txline[Interface][7 + 2 * sizeof (Vec3D_t) + sizeof (REAL)], sizeof (REAL));
                        memcpy (&EEPROM_emul_DataTemp.ADCSConfig.determinationConfig.satAvgDragArea, (REAL *)&txline[Interface][7 + 2 * sizeof (Vec3D_t) + 2 * sizeof (REAL)], sizeof (REAL));
                        /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
                        EEPROM_Emul_SyncInfo();
                        /* Inform the user */
                        sprintf (print_buff[Interface], "OK ADCS Determination Setup complete");
                        ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        break;
                    case ESTTC_CMD_SET_ADCS_FORCE: // 0x41
                        if ((uint8_t)txline[Interface][7] > ADCS_STATE_TARGETING_POS) {

                        }
                        if (ADCS_OK == ADCS_SetState ((ADCS_state_t)txline[Interface][7], NULL, 0)) {
                            sprintf (print_buff[Interface], "OK ADCS FORCED STATE %d", (int)txline[Interface][7]);
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        } else {
                            sprintf (print_buff[Interface], "ERR - executing");
                            ESTTC_CMD_CRC_Print_string (ComInterface, print_buff[Interface]);
                        }
                        break;
#endif /* ENABLE_OBC_ADCS */

                    case ESTTC_CMD_ANT_SETTINGS: //0x47:	// Next power up will be managed like it is first try to deploy the antenna

                        if( txline[Interface][9] < ANTUHF_POWER_UP_MIN_TIME )
                        {
                            sprintf(print_buff[Interface], "Wrong power-up delay");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }else{
                            EEPROM_emul_DataTemp.AntUHFSettings.u32Settings = (uint32_t)((uint32_t)txline[Interface][7] << 24) |
                                                                                        ((uint32_t)txline[Interface][8] << 16) |
                                                                                        ((uint32_t)txline[Interface][9] <<  8) |
                                                                                        ((uint32_t)txline[Interface][10]);
                            /* Change the delay right away */
                            AntUHF_ChangePowerUpDelay(txline[Interface][9]);

                            /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
                            EEPROM_Emul_SyncInfo();

                            /* Print out the status of bit 1 - Enabled/Disable of the Antenna service */
                            sprintf(print_buff[Interface], "OK:\r\nUHF Antenna service: ");
                            if( txline[Interface][10] & 0x01 )
                            {
                                sprintf(&print_buff[Interface][26], "Enabled");
                            }else{
                                sprintf(&print_buff[Interface][26], "Disabled");
                            }
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                            /* Print out the status of First deployment algorithm */
                            sprintf(print_buff[Interface], "Special algorithm: ");
                            if( txline[Interface][10] & 0x02 )
                            {
                                sprintf(&print_buff[Interface][19], "Enabled");
                            }else{
                                sprintf(&print_buff[Interface][19], "Disabled");
                            }
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                            /* Print out the delay after power up until the service of the antenna is started */
                            sprintf(print_buff[Interface], "Power up delay: %d Minutes", txline[Interface][9]);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                        break;

                    case ESTTC_CMD_RST_COUNTS:  // 0x61
                    {
                        switch( txline[Interface][7] )
                        {
                            case 0:
                            {
                                BootData->RST_WWD = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: WWD");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 1:
                            {
                                BootData->RST_IWD = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: IWD");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 2:
                            {
                                BootData->RST_LPR = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: LPR");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 3:
                            {
                                BootData->RST_POR = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: POR");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 4:
                            {
                                BootData->RST_RstPin = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: RstPin");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 5:
                            {
                                BootData->RST_BOR = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: BOR");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 6:
                            {
                                BootData->RST_HardFault = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: HardFault");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 7:
                            {
                                BootData->RST_MemFault = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: MemFault");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 8:
                            {
                                BootData->RST_BusFault = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: BusFault");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            case 9:
                            {
                                BootData->RST_UsageFault = 0;
                                sprintf(print_buff[Interface], "Cleared RST counter: UsageFault");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }break;

                            default:
                                sprintf(print_buff[Interface], "Wrong reset counter number: %0d", txline[Interface][7]);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                break;
                        }

                        RTC_SetRTC_BKP_CRC();

                        /* Sync the RTC backup registers data with the buffer of the emulated EEPROM memory */
                        memcpy(&EEPROM_emul_DataTemp.ResetFaults, (void *)BootData, sizeof(EEPROM_emul_DataTemp));

                        /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
                        EEPROM_Emul_SyncInfo();

                    }break;

                    case ESTTC_CMD_FAULTS_TST:  // 0x62
                    {
#ifdef ESTTC_FAULT_TESTS_ENABLE
                        switch( txline[Interface][7] )
                        {
                            case 0:// WWD reset
                            {
                                HAL_WWDG_Refresh(&hwwdg);
                            }break;

                            case 1:// IWD reset
                            {
                                hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
                                hiwdg.Init.Reload = 1;
                                if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
                                {
                                    Error_Handler();
                                }
                            }break;

                            case 2: //Reserved for HardFault - SVC CALL or any other
                            {

                            }break;

                            case 3:  // Memory fault
                            {
                                typedef void (*pFunction)(void);

                                /* branch into the System region (XN region) (0xE0000000 - 0xFFFFFFFF) */
                                pFunction appEntry;
                                appEntry = (pFunction)0xFFFFFFFF;
                                appEntry();
                            }break;

                            case 4:
                            {
                                #define ESTTC_CPU_INVALID_ADDRESS       (0x44000000)

                                uint8_t * IllegalVector  = (uint8_t *)ESTTC_CPU_INVALID_ADDRESS;  // Bus fault

                                *IllegalVector = 0;
                            }break;

                            case 5:
                            {
                                SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;     // Enable zero devision fault

                                int a = 10;
                                int b = 0;
                                int c;
                                c = a/b; // Usage fault
                                (void)c;

                            }break;

                            default:
                                sprintf(print_buff[Interface], "Wrong fault number: %0d", txline[Interface][7]);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                break;
                        }
#else   /* ESTTC_FAULT_TESTS_ENABLE */
                        sprintf(print_buff[Interface], "The command is disabled");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
#endif   /* ESTTC_FAULT_TESTS_ENABLE */
                    }break;

                    case ESTTC_CMD_RESET: // Conditional RESET
#ifndef NO_BOOTLOADER_ENABLED
                      if ((txline[Interface][6] == 1) && (txline[Interface][7] == 0xA))
                      {
                        sprintf(print_buff[Interface], "OK+APPL");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        BootData->Mailbox = MAILBOX_VAL_APPL; //*((__IO uint32_t *)MAILBOX_ADDRESS) = MAILBOX_VAL_APPL;

                        __disable_interrupt();
                        SCB->VTOR = APPL_ADDRESS;
                      }
                      else
                      if ((txline[Interface][6] == 1) && (txline[Interface][7] == 0xB))
                      {
                        sprintf(print_buff[Interface], "OK+BOOT");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        BootData->Mailbox = MAILBOX_VAL_BOOT; //*((__IO uint32_t *)MAILBOX_ADDRESS) = MAILBOX_VAL_BOOT;

                        __disable_interrupt();
                        SCB->VTOR = BOOT_ADDRESS;
                      }
                      else{
                        sprintf(print_buff[Interface], "ERR");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }
                      RTC_SetRTC_BKP_CRC();    /* set valid checksum for the RTC backup registers after changing the mailbox */
#else
//The implementation of that command when the bootloader is not used is completely up to the user. Only reset will be executed by default.
#endif
                      osDelay(50);             /* wait some time until the USART buffers are empty */
                      // Reset the OBC
                      TaskMonitor_ImmediatReset(TASK_MONITOR_SW_RESET);
                      break;

                    default:{
                      sprintf(print_buff[Interface], "ERR - parameter");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                      break;
                  } // switch case
              } /* if (txline[Interface][5] <= ESTTC_CMD_PAR_NUM) */
              else{
                  sprintf(print_buff[Interface], "ERR - parameter");
                  ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
              }
            } /* if (txline[Interface][4] == OBC_I2C_ADDRESS) */
            else
            if (txline[Interface][4] == UHF_I2C_ADDRESS)
            {
              uint8_t TxSize = txline[Interface][5];

              if ((TxSize + 2) == (len-8))
              {
                  uint8_t timeout = 0;
                  HAL_StatusTypeDef I2C_retStat;
                  uint8_t RxSize = txline[Interface][6];

                  CRC_value_calc = crc32(0, (BYTE *)&rxline[Interface][10], TxSize);   /* Calculate the CRC */
                  sprintf(&rxline[Interface][10+TxSize], " %08X\r", (unsigned int)CRC_value_calc);   /* Attach the calculated CRC at the end of the string */
                  TxSize += ESTTC_CYMBOLS_IN_CRC+1;

                  MX_I2C1_Init(); //Enable I2C1 interface
                  do
                  {
                    

                    if( timeout >= 1 )
                    {
                        // if there is any error reset the I2C interface
                        if(( I2C_retStat == HAL_ERROR )||( I2C_retStat == HAL_TIMEOUT)||(( I2C_retStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY)))
                        {
                            I2C_Reset(&hi2c1);
                        }
                        osDelay(5);

                        if( timeout >= 10 )
                        {
                            // Stop trying after certain times
                            break;
                        }
                    }

                    I2C_retStat = HAL_I2C_Master_Transmit(&hi2c1, txline[Interface][9]<<1, (uint8_t *)&rxline[Interface][10], TxSize, 150);

                    if (HAL_OK == I2C_retStat)
                    {
                        sprintf(print_buff[Interface], "OK UHF CMD %X", txline[Interface][6+4]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                        //Just in case the read length is less then the full string add new line and zero termination
                        rxline[Interface][RxSize] = '\r'; // add new line
                        rxline[Interface][RxSize+1] = 0;  // add zero termination

                        I2C_retStat = HAL_I2C_Master_Receive(&hi2c1, txline[Interface][9]<<1, (uint8_t *)&rxline[Interface][0], RxSize, 150);

                        if(rxline[Interface][RxSize-1] == '\r')
                        {
                            rxline[Interface][RxSize] = 0;  // add zero termination
                        }

                        if (HAL_OK == I2C_retStat)
                        {
                            fprintf(ComInterface, "UHF: %s", &rxline[Interface][0]);
                        }else{
                            sprintf(print_buff[Interface], "ERR - Rx");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }
                    }
                    else{
                      sprintf(print_buff[Interface], "ERR - Tx");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }

                    timeout ++; //count one more try
                }while( I2C_retStat  != HAL_OK);
				
				HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface
              }
              else{
                  sprintf(print_buff[Interface], "ERR - length");
                  ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
              }
            }
            else{
              sprintf(print_buff[Interface], "ERR addr");
              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
            }
          }
          else if(txline[Interface][3] == 'D')
          {
              if( (txline[Interface][4] == S_X_BAND_ADDRESS) ||                 /* All X-band and S-band transmitters commands */
                  ((txline[Interface][4] == OBC_I2C_ADDRESS)&&(begin[6] == 'F') /* All OBC file commands */
                    &&(begin[7] != 'W')) )                                      /* the write command will receive more bytes so the CRC will be checked after that */
              {
                  /* get CRC */
                  int ch = -1;
                  char numbCR = 0;

                  for (i = 0; i < 9; i++)
                  {
                      if ((ch = getbyte(20, Interface)) == -1){
                        break;
                      }

                      begin[len+1+i] = (BYTE)ch;    /* Place the expected CRC after the the Carriage Return (0x0D) */

                      if( ch == 0x0D )
                      {
                        numbCR ++;    /* Count the number of the Carriage return */
                      }
                  }

                  if( i == 9 )
                  {
                      CRC_value_calc = crc32(0, (BYTE *)begin, len);

                      CRC_value_rx = ESTTC_ExtractCRC(&begin[len+2]);

                      if( CRC_value_calc != CRC_value_rx )
                      {
                          sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          return ProcessedPacket;
                      }
                  }else{
                      i -= numbCR;

                      if( i != 0 )    /* there should be none data but Carriage return symbols */
                      {
                          sprintf(print_buff[Interface], "ERR - length");
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                          return ProcessedPacket;
                      }
                  }
              }

            if(txline[Interface][4] == S_X_BAND_ADDRESS)
            {
                uint8_t  data_size;
                uint16_t Identifier = (txline[Interface][7]<<8) + txline[Interface][8];
                SX_BAND_Selection_Enum SX_BAND_Select_Module;

                if((Identifier >= S_BAND_ID_RANGE_MIN)&&(Identifier <= S_BAND_ID_RANGE_MAX))
                {
                    data_size  = txline[Interface][6] - S_BAND_ID_LENGTH;
                    SX_BAND_Select_Module = SX_SELECT_S_BAND_TRANSCEIVER;

                    sprintf(print_buff[Interface], "Requesting S-Band CMD 0x%X ... ",txline[Interface][5]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }else if((Identifier >= X_BAND_ID_RANGE_MIN)&&(Identifier <= X_BAND_ID_RANGE_MAX))
                {
                    data_size  = txline[Interface][6] - X_BAND_ID_LENGT;
                    SX_BAND_Select_Module = SX_SELECT_X_BAND_TRANSCEIVER;

                    sprintf(print_buff[Interface], "Requesting X-Band CMD 0x%X ... ",txline[Interface][5]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }else{
                    sprintf(print_buff[Interface], "Err - Unknown ID %X", Identifier);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }

                switch( txline[Interface][5] )
                {
                    case 0x55:
                    {
                        // 0 byte
                        if( txline[Interface][6] == S_BAND_ID_LENGTH )
                        {
                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', 0, NULL);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', 0, NULL);
                            }
                        }else{
                            sprintf(print_buff[Interface], "Err - Invalid length of the CMD");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }break;

                    case 0x35:
                    case 0x50:
                    case 0x60:
                    case 0x61:
                    case 0x70:
                    {
                        // File names with length between 3 and 30 characters
                        if(( txline[Interface][6] >= (S_BAND_ID_LENGTH+3) )&&( txline[Interface][6] <= (S_BAND_ID_LENGTH+30) ))
                        {
                            if( SX_BAND_Select_Module == SX_SELECT_S_BAND_TRANSCEIVER )
                            {
                                S_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&begin[14]);
                            }else{
                                X_BAND_TRNSM_ESTTC_StartCmd(Identifier, txline[Interface][5], 'W', data_size, (uint8_t *)&begin[14]);
                            }
                        }else{
                            sprintf(print_buff[Interface], "Err - Invalid length of the CMD");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }break;

                    default:
                    {
                        sprintf(print_buff[Interface], "Err - Invalid CMD");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
            }else if(txline[Interface][4] == OBC_I2C_ADDRESS)
            {
              if (begin[6] == 'F')
              {
                switch(begin[7])
                {
                    case 'B':
                    {
                        sprintf(print_buff[Interface], "ERR+INAPP");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    break;

                    case 'A':
                    {
                        sprintf(print_buff[Interface], "ERR+INAPP");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    break;

                    case 'F': /* Format the SD Cards */
                    {
                        if( 0 == memcmp("SDCard", &begin[8], 6 ) )
                        {
                            fr = f_mkfs("0", FM_FAT, 0, NULL, 0);

                            if( FR_OK == fr )
                            {
                                sprintf(print_buff[Interface], "OK+Format OK. SD card is empty.");
                            }else{
                                sprintf(print_buff[Interface], "ERR+Format failed(%u)", fr);
                            }
                        }else{
                            sprintf(print_buff[Interface], "ERR+Wrong PSW");
                        }
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }break;

                  case 'C': /* Close a file */
                    if (df[Interface].obj.fs)
                    {
                        fr = f_close(&df[Interface]);
						if( FR_OK == fr )
						{
						    sprintf(print_buff[Interface], "OK Closed File");
						    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
						}else{
						    sprintf(print_buff[Interface], "ERR+FNF(%u)", fr);
						    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
						}
                    }else{
                        sprintf(print_buff[Interface], "OK Nothing to Close");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    ESTTC_sync_file_timeout[Interface] = 0; /* sync of the file is done here, no need to do it afterwards at timeout */
                    break;

                  case 'S': // Calculate file checksum
                      if (df[Interface].obj.fs)
						f_close(&df[Interface]);
                    sprintf(txline[Interface], "0:/%s", &begin[8]);
                    if (FR_OK != (fr = f_open(&df[Interface], txline[Interface], FA_READ | FA_OPEN_EXISTING)))
                    {
                      sprintf(print_buff[Interface], "ERR+FNF(%u)=%s", fr, txline[Interface]);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }else if(fr == FR_DISK_ERR){
                        // on disk error - try to reinitialise the SD card
                        InitCdCard();
                    }

                    for (j = 0, i = 0; i < f_size(&df[Interface]); i++)
                    {
                      if (FR_OK != (fr = f_read(&df[Interface], &s8_tmp, 1, (UINT*)&br)))
                      {
                        f_close(&df[Interface]);
                        sprintf(print_buff[Interface], "ERR+FIR(%u)=%u", (uint16_t)fr, (uint16_t)br);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        break;
                      }
                      if (1 != br)
                      {
                        f_close(&df[Interface]);
                        sprintf(print_buff[Interface], "ERR+FRS=%u", (uint16_t)br);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        break;
                      }
                      j += s8_tmp;
                    }

                    if (FR_OK == f_stat(txline[Interface], &fno[Interface]) ) 
                    {
                        sprintf(&print_buff[Interface][0], "OK+%04X", (uint16_t)(j>>16));
                        sprintf(&print_buff[Interface][3+4], "%04X", (uint16_t)j);
                        sprintf(&print_buff[Interface][3+4+4], " %04X", (uint16_t)(fno[Interface].fsize>>16));
                        sprintf(&print_buff[Interface][3+4+4+5], "%04X", (uint16_t)fno[Interface].fsize);

                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                    }else{
                        sprintf(print_buff[Interface], "ERR+WrongPath");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }

                    f_close(&df[Interface]);
                    break;

                  case 'I': // Find file for reading
                      if (df[Interface].obj.fs)
                        f_close(&df[Interface]);
                    sprintf(txline[Interface], "0:/%s", &begin[8]);
                    if (FR_OK == (fr = f_open(&df[Interface], txline[Interface], FA_READ | FA_OPEN_EXISTING)))
                    {
                        if (FR_OK == (fr = f_stat(txline[Interface], &fno[Interface])) )
                        {
                            sprintf(&print_buff[Interface][0], "OK+%04X", (uint16_t)(fno[Interface].fsize>>16));
                            sprintf(&print_buff[Interface][7], "%04X Found+Opened", (uint16_t)fno[Interface].fsize);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }else{
                            sprintf(print_buff[Interface], "ERR+FNF(%u)=%s", fr, txline[Interface]);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }else {
                        if(fr == FR_DISK_ERR){
                            // on disk error - try to reinitialise the SD card
                            InitCdCard();
                        }

                        sprintf(print_buff[Interface], "ERR+FNF(%u)=%s", fr, txline[Interface]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    break;

                  case 'R': // Read from file @ position
                      if (df[Interface].obj.fs == 0)
                    {
                      sprintf(print_buff[Interface], "ERR+FIH");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    sscanf(&begin[8], "%08X", (unsigned int *)&fpos[Interface]);
                    sscanf(&begin[8+8], "%04X", (unsigned int *)&bsize[Interface]);
                    if ((fpos[Interface] > f_size(&df[Interface]))||(bsize[Interface] > sizeof(txline[Interface])))
                    {
                        sprintf(&print_buff[Interface][0], "ERR+FIP=");
                        sprintf(&print_buff[Interface][8], "%04X",(uint16_t)(fpos[Interface]>>16));
                        sprintf(&print_buff[Interface][8+4], "%04X",(uint16_t)fpos[Interface]);

                        sprintf(&print_buff[Interface][8+4+4], "-%04X",(uint16_t)(fno[Interface].fsize>>16));
                        sprintf(&print_buff[Interface][8+4+4+5], "%04X,",(uint16_t)fno[Interface].fsize);

                        sprintf(&print_buff[Interface][8+4+4+5+5], "%04X",(uint16_t)(bsize[Interface]>>16));
                        sprintf(&print_buff[Interface][8+4+4+5+5+4], "%04X",(uint16_t)bsize[Interface]);

                        sprintf(&print_buff[Interface][8+4+4+5+5+4+4], "-%04X",(uint16_t)(sizeof(txline[Interface])>>16));
                        sprintf(&print_buff[Interface][8+4+4+5+5+4+4+5], "%04X",(uint16_t)sizeof(txline[Interface]));

                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                      break;
                    }
                    if (FR_OK != (fr = f_lseek(&df[Interface], fpos[Interface])))
                    {
                      sprintf(print_buff[Interface], "ERR+FIS(%u)=%u", (uint16_t)fr, (uint16_t)fpos[Interface]);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    if (FR_OK != (fr = f_read(&df[Interface], txline[Interface], bsize[Interface], (UINT*)&br)))
                    {
                      sprintf(print_buff[Interface], "ERR+FIR(%u)=%u", fr, (uint16_t)br);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    if (bsize[Interface] != br)
                    {
                      sprintf(print_buff[Interface], "ERR+FRS(%u)=%u", (uint16_t)bsize[Interface], (uint16_t)br);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    br = 0;
                    for (i = 0; i < bsize[Interface]; i++)
                    {
                      br += txline[Interface][i];
                      sprintf(&print_buff[Interface][i], "%c", txline[Interface][i]);
                    }

                    sprintf(&print_buff[Interface][bsize[Interface]], "%c", (BYTE)br);

                    ESTTC_CMD_CRC_Print_raw_data(ComInterface, print_buff[Interface], bsize[Interface]+1);

                    break;

                  case 'O': // Create file for writing
                      if (df[Interface].obj.fs)
					  	f_close(&df[Interface]);
                      sprintf(txline[Interface], "0:/%s", &begin[8]);
                      if (FR_OK == (fr = f_open(&df[Interface], txline[Interface], FA_WRITE | FA_CREATE_ALWAYS)))
                      {
                          if (FR_OK ==( fr = f_stat(txline[Interface], &fno[Interface]) ) )
                          {
                              sprintf(print_buff[Interface], "OK+Created+Opened %s", txline[Interface]);
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }else{
                              sprintf(print_buff[Interface], "ERR+FNC(%u)=%s", fr, txline[Interface]);
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }
                      }else{
                          if(fr == FR_DISK_ERR){
                              // on disk error - try to reinitialise the SD card
                              InitCdCard();
                          }

                          sprintf(print_buff[Interface], "ERR+FNC(%u)=%s", fr, txline[Interface]);
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }

                      ESTTC_sync_file_numbPackets[Interface] = 0;   /* Start over to count number of written packets in one file */
                      break;

                  case 'E': // Open existing file for writing
                      if (df[Interface].obj.fs)
                          f_close(&df[Interface]);
                      sprintf(txline[Interface], "0:/%s", &begin[8]);
                      if (FR_OK == (fr = f_open(&df[Interface], txline[Interface], FA_WRITE | FA_OPEN_EXISTING)))
                      {
                          if (FR_OK ==( fr = f_stat(txline[Interface], &fno[Interface]) ) )
                          {
                              sprintf(print_buff[Interface], "OK Open %s", txline[Interface]);
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }else{
                              sprintf(print_buff[Interface], "ERR+FNC(%u)=%s", fr, txline[Interface]);
                              ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                          }
                      }else{
                          if(fr == FR_DISK_ERR){
                              // on disk error - try to reinitialise the SD card
                              InitCdCard();
                          }
                          sprintf(print_buff[Interface], "ERR+FNC(%u)=%s", fr, txline[Interface]);
                          ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      }

                      ESTTC_sync_file_numbPackets[Interface] = 0;   /* Start over to count number of written packets in one file */
                      break;

                  case 'W': // Write to file @ position
                  {
                      if (df[Interface].obj.fs == NULL)
                    {
                      sprintf(print_buff[Interface], "ERR+FIH");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    sscanf(&begin[8], "%08X", (unsigned int *)&fpos[Interface]);
                    sscanf(&begin[8+8], "%04X", (unsigned int *)&bsize[Interface]);
                    if ((fpos[Interface] > f_size(&df[Interface]))||(bsize[Interface] > sizeof(txline[Interface])))
                    {
                        sprintf(&print_buff[Interface][0], "ERR+FIP=");

                        sprintf(&print_buff[Interface][8], "%04X",(uint16_t)(fpos[Interface]>>16));
                        sprintf(&print_buff[Interface][8+4], "%04X",(uint16_t)fpos[Interface]);

                        sprintf(&print_buff[Interface][8+4+4], "-%04X",(uint16_t)(fno[Interface].fsize>>16));
                        sprintf(&print_buff[Interface][8+4+4+5], "%04X,",(uint16_t)fno[Interface].fsize);

                        sprintf(&print_buff[Interface][8+4+4+5+5], "%s",&begin[8]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    if (FR_OK != (fr = f_lseek(&df[Interface], fpos[Interface])))
                    {
                      sprintf(print_buff[Interface], "ERR+FIS(%u)=%u", (uint16_t)fr, (uint16_t)fpos[Interface]);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }

                            begin[len] = '\r'; /* restore the '\r' symbol */
                            CRC_value_calc = crc32(0, (BYTE *)begin, len+1); /* calculate the CRC over the packet without the data that are going to be written */

                    int ch = -1;
                    br = 0;
                    for (i = 0; i < bsize[Interface]; i++)
                    {
                      if ((ch = getbyte(1200, Interface)) == -1) break; /* receive the data that are going to be written */
                      rxline[Interface][i] = (BYTE)ch;
                      br += (BYTE)ch;
                    }

                    if (ch == -1)
                    {
                      sprintf(print_buff[Interface], "ERR+FTM");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }

                    if ((ch = getbyte(40, Interface)) == -1) /* receive the checksum */
                    {
                      sprintf(print_buff[Interface], "ERR+FTM");
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    rxline[Interface][i] = ch; /* add the checksum to the buffer */


                    if (((BYTE)br) != ((BYTE)ch)) /* compare the calculated checksum and the received checksum */
                    {
                      sprintf(print_buff[Interface], "ERR+FEC=%02X(%02X)", (BYTE)br, (BYTE)ch);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }

                    char numbCR = 0;

                    for (i = 0; i < 9; i++) /* read space + 8 symbols for CRC value */
                    {
                      if ((ch = getbyte(80, Interface)) == -1){ /* read a byte from the USART buffer */
                          break;
                      }

                      txline[Interface][i] = (BYTE)ch; /* store the byte in the buffer */

                      if( ch == 0x0D )
                      {
                        numbCR ++;    /* Count the number of the Carriage return */
                      }
                    }

                    if( i == 9 )
                    {
                        CRC_value_calc = crc32((DWORD)CRC_value_calc, (BYTE *)&rxline[Interface][0], bsize[Interface]+1); /* calculate the CRC over the rest of the packet */

                        CRC_value_rx = ESTTC_ExtractCRC(&txline[Interface][1]);

                        if( CRC_value_calc != CRC_value_rx )    /* compare the calculated value over the packet with the receive value at the end of the packet */
                        {
                            sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }
                    }else
                    {
                        i -= numbCR;

                        if( i != 0 )
                        {
                            sprintf(print_buff[Interface], "ERR - length");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                            return ProcessedPacket;
                        }
                    }

                    if (FR_OK != (fr = f_write(&df[Interface], rxline[Interface], bsize[Interface], (UINT*)&br)))
                    {
                      sprintf(print_buff[Interface], "ERR+FWE=%u", fr);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    if (bsize[Interface] != br)
                    {
                      sprintf(print_buff[Interface], "ERR+FWC=%u(%u)", (uint16_t)br, (uint16_t)bsize[Interface]);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }

                    ESTTC_sync_file_timeout[Interface] = ESTTS_SYNC_FILE_TIMEOUT;   /* timeout to sync the file if there is no writing request for long time */
                    ESTTC_sync_file_numbPackets[Interface] ++;                      /* count number of written packets */

                    sprintf(print_buff[Interface], "OK Written");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }break;

                  case 'D': // Delete file
                      if (df[Interface].obj.fs)
						f_close(&df[Interface]);
                    sprintf(txline[Interface], "0:/%s", &begin[8]);
                    if (FR_OK != f_unlink(txline[Interface])){
                      sprintf(print_buff[Interface], "ERR+FDL%s", txline[Interface]);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }else{
                        sprintf(print_buff[Interface], "OK Deleted %s", txline[Interface]);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                    break;

                  case 'L': // Write to file "DirList" a list of existing files
                      if (df[Interface].obj.fs)
						f_close(&df[Interface]);
                    if (FR_OK != (fr = f_open(&df[Interface], "0:/DirList.txt", FA_WRITE | FA_CREATE_ALWAYS)))
                    {
                        if(fr == FR_DISK_ERR){
                            // on disk error - try to reinitialise the SD card
                            InitCdCard();
                        }

                        sprintf(print_buff[Interface], "ERR+LCF(%u)", fr);
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    len = strlen(&begin[8]);
                    if ((len == 0) || (len > 12))
                    {
                      begin[8] = '*';
                      begin[9] = 0;
                    }
                    fprintf((FILE *)&df[Interface], "----- FATFS RevID.%05u -----\r\n", _FATFS);
                    fprintf((FILE *)&df[Interface], "--- Name ---    --- size ---\r\n" );
                    strcpy(txline[Interface], &begin[8]);
                    j = 0;
                    if (FR_OK != (fr = f_findfirst(&dd[Interface], &fno[Interface], "", txline[Interface])))
                    {
                      sprintf(print_buff[Interface], "ERR+LFF(%u)", fr);
                      ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                      break;
                    }
                    do {
                      if (strlen(fno[Interface].fname) == 0) break;
                      j++;

                      if( fno[Interface].fsize < 0xFFFF )
                      {
                          sprintf(rxline[Interface], "%13s   %u B\n", fno[Interface].fname, (uint16_t)fno[Interface].fsize);
                      }else if( fno[Interface].fsize < 0x3FFFFF )
                      {
                          sprintf(rxline[Interface], "%13s   ~%u kB\n", fno[Interface].fname, (uint16_t)(fno[Interface].fsize>>10));
                      }else
                      {
                          sprintf(rxline[Interface], "%13s   ~%u MB\n", fno[Interface].fname, (uint16_t)(fno[Interface].fsize>>20));
                      }

                      f_puts(rxline[Interface], &df[Interface]);
                    } while (FR_OK == f_findnext(&dd[Interface], &fno[Interface]));
                    fprintf((FILE *)&df[Interface], "-----------------------------\n         TOTAL FILES %u\n", (uint16_t)j);
                    f_close(&df[Interface]);
                    sprintf(print_buff[Interface], "OK DirList %s", txline[Interface]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    break;

                  default: // Unknown command
                    sprintf(print_buff[Interface], "ERR+UNC:%s", &begin[7]);
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    break;
                }
              }
			  else{
                  sprintf(print_buff[Interface], "ERR Wrong Data type");
                  ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }
            }
			else{
                sprintf(print_buff[Interface], "ERR Address");
                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
              }
          }
          else{
            sprintf(print_buff[Interface], "ERR cmd");
            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
          }
        }else{
            sprintf(print_buff[Interface], "ERR - Wrong Length!");
            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        }
      }
    }else{
        /* Check if a file has been written but has not been closed for a long time */
        if( ESTTC_sync_file_timeout[Interface] )
        {
            ESTTC_sync_file_timeout[Interface] --;

            if(( ESTTC_sync_file_timeout[Interface] == 1 )||( ESTTC_sync_file_numbPackets[Interface] > ESTTS_SYNC_FILE_PACKETS ))
            {
                ESTTC_sync_file_numbPackets[Interface] = 0;

                if (df[Interface].obj.fs)   /* if there is a open file */
                {
                    /* sync the data */
                    fr = f_sync(&df[Interface]);
                }
            }
        }
    }

    return ProcessedPacket;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Gets a single character from UART/USART serial buffer
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static int ESTTC_getchar(ESTTC_InterfacesEnum Interface)
{
    int ch = -1;

    __disable_interrupt();

    if(RxBuffTail[Interface] != RxBuffHead[Interface])
    {
        ch = (BYTE)RxBuffer[Interface][RxBuffTail[Interface]];
        RxBuffTail[Interface] = (RxBuffTail[Interface] + 1) % UART_BUFFER_SIZE;
        RxBuffLen[Interface]--;
    }
    __enable_interrupt();

    return ch;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Gets a single byte from UART/USART serial buffer in blocking manner
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static int getbyte(uint32_t tmt_ms, ESTTC_InterfacesEnum Interface)
{
  int ch;

  if ((ch = ESTTC_getchar(Interface)) != -1) return ch;

  for (uint32_t i = 0; i < tmt_ms; i++)
  {
    osDelay(1);
    if ((ch = ESTTC_getchar(Interface)) != -1) break;
  }
  return ch;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Gets a whole phrase from UART/USART serial buffer in blocking manner
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint32_t GetPhrase(char *dst, uint32_t len, char term, ESTTC_InterfacesEnum Interface)
{
  int ch;

  if ((dst == NULL) || (len == 0)) return 0;

  while((ch = ESTTC_getchar(Interface)) != -1 )
  {
      dst[pack_data_position[Interface]] = (BYTE)ch;

      pack_data_position[Interface] = (pack_data_position[Interface] + 1) % LINE_BUFFER_SIZE;

      if(term == (BYTE)ch)
      {
          dst[pack_data_position[Interface]-1] = 0;
          pack_data_position[Interface] = 0;
          return 1;
      }
  }

  return 0;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Hexadecimal ASCII to binary number converted
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      hb - high significant part of the byte
*                    lb - low significant part of the byte
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint8_t HexToBin(uint8_t hb, uint8_t lb)
{
  uint8_t thb = hb, tlb = lb;

  if (thb > '9') thb += 9;
  if (tlb > '9') tlb += 9;

  return (thb << 4) + (tlb & 0x0f);
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Gets CMD interface command length
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      cmd_type - type of the command:
                              0 - Read command
                              1 - Write command
* @param[input]      CMD - Command
* @param[output]     none
* @return            uint32_t - CRC value
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint16_t ESTTC_GetCmdLength( uint8_t cmd_type , uint8_t CMD )
{
    uint16_t                cmd_length = 0;
    uint8_t                 u8_index;
    uint8_t                 cmd_number = 0;
    esttc_cmd_params_type * temp_cmd_params;

    if( cmd_type == 0 ) /* if the command is type Reading */
    {
        temp_cmd_params = (esttc_cmd_params_type *)ESTTC_ReadCmdLenght;
        cmd_number = ESTTC_NUMBER_READ_CMDS;
    }else if( cmd_type == 1 ) /* if the command is type Reading */
    {
        temp_cmd_params = (esttc_cmd_params_type *)ESTTC_WriteCmdLenght;
        cmd_number = ESTTC_NUMBER_WRITE_CMDS;
    }
    else if( cmd_type == 2 ) /* if the command is type Reading S or Xband */
    {
        return 12;
    }
    else if( cmd_type == 3 ) /* if the command is type Writing S or Xband  */
    {
        switch( CMD )
        {
            case 0x01:
            case 0x02:
            case 0x04:
            case 0x05:
            case 0x06:
            case 0x07:
            case 0x0C:
            {
                // one byte
                return 10 + 4 + 2;  //Header + ID + data
            }break;

#ifdef ENABLE_SX_BAND_TESTBOARD
            case 0x0D:
            {
                // one byte
                return 10 + 4 + 168;
            }break;
#endif /* #ifdef ENABLE_SX_BAND_TESTBOARD */

            case 0x30:
            case 0x31:
            case 0x32:
            {
                // 0 byte
                return 10 + 4;
            }break;

            case 0x03:
            {
                return 10 + 4 + 8;
            }break;

            case 0x08:
            {
                return 10 + 4 + 4;
            }break;

            case 0x09:
            {
                return 10 + 4 + 24;
            }break;

            default:
            {
                return 0;
            }
        }
    }else{
        return 0;
    }

    for( u8_index = 0; u8_index < cmd_number; u8_index ++ )
    {
        if(temp_cmd_params[u8_index].cmd == CMD)
        {
            cmd_length = temp_cmd_params[u8_index].length;
            break;
        }
    }

    return cmd_length;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Converts ASCII string of a CRC value to a hex value
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      crc_buffer - Array with a ASCII string with CRC
* @param[output]     none
* @return            uint32_t - CRC value
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint32_t ESTTC_ExtractCRC(char * crc_buffer)
{
    uint8_t i , j;
    uint8_t CRC_value_rx_buf[4];
    uint32_t CRC_value_rx;

    /* convert the CRC from ASCII to hex values */
    for (i = 0, j = 0; i < 1+8; i+=2, j++)
    {
        CRC_value_rx_buf[j] = HexToBin(crc_buffer[i], crc_buffer[i+1]);
    }

    /* combine the four bytes to one word */
    CRC_value_rx =  CRC_value_rx_buf[0] << 24 |
                    CRC_value_rx_buf[1] << 16 |
                    CRC_value_rx_buf[2] <<  8 |
                    CRC_value_rx_buf[3];

    return CRC_value_rx;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Validate the inserted date
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      year  - Array with a ASCII string with CRC
* @param[input]      month - Array with a ASCII string with CRC
* @param[input]      day   - Array with a ASCII string with CRC
* @param[output]     none
* @return            uint32_t - CRC value
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static int ESTTC_CheckDate(int year, int month, int day )
{
    uint8_t MaxDay;

    if((month > 12)||(month < 1)||(day < 1))
    {
        /* Wrong month or day */
        return 0;
    }

    if ((month == 1) || (month == 3) || (month == 5) || (month == 7) ||
        (month == 8) || (month == 10) || (month == 12))
    {
        MaxDay = 31;
    }
    else if ((month == 4) || (month == 6) || (month == 9) || (month == 11))
    {
        MaxDay = 30;
    }
    else if ((month == 2) && (year % 4 == 0))
    {
        MaxDay = 29;
    }
    else if ((month == 2) && (year % 4 != 0))
    {
        MaxDay = 28;
    }else {
        /* Wrong month */
        return 0;
    }

    if( MaxDay < day )
    {
        /* Wrong day */
        return 0;
    }

    /* Valid date */
    return 1;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
