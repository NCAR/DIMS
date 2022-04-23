/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file Svc_RTC.h
* @brief Header of Svc_RTC.c
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Vasil Milev
* @version           1.0.0
* @date              2019.05.15
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.05.15, author Vasil Milev, Initial revision }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef SVC_RTC_H
#define SVC_RTC_H

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "MCU_Init.h"
#include "User_types.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* That period should be adjusted according the temperature simulations of the OBC. For accurate
 * RTC it is recommended to do calibration if the temperature change more then 3 degrees.
 * If according to the simulations the temperature changes with 3 degrees per 10 minutes,
 * the minimum calibration period should be 10 minutes */
#define RTC_CALIBRATION_PERIOD  ((3*60*1000)/SERVICE_TASK_PERIOD) /* 3 minutes - expressed in ms */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

typedef struct {
  uint32_t Reserve1;
  uint32_t Mailbox;          /* Keeps magic number giving info for last run SW and reset reason */

  uint32_t RST_WWD;          /* WINDOW_WATCHDOG_RESET */
  uint32_t RST_IWD;          /* INDEPENDENT_WATCHDOG_RESET */
  uint32_t RST_LPR;          /* LOW_POWER_RESET */
  uint32_t RST_POR;          /* POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR) */
  uint32_t RST_RstPin;       /* EXTERNAL_RESET_PIN_RESET */
  uint32_t RST_BOR;          /* BROWNOUT_RESET (BOR) */

  uint32_t RST_HardFault;    /* HardFault_Handler()  */
  uint32_t RST_MemFault;     /* MemManage_Handler()  */
  uint32_t RST_BusFault;     /* BusFault_Handler()   */
  uint32_t RST_UsageFault;   /* UsageFault_Handler() */

  uint32_t RST_ErrHandler;   /* UsageFault_Handler() */

  uint32_t Reserve2;         /* reserved */
  uint32_t Reserve3;         /* reserved */
  uint32_t BootDataCRC;
} boot_struct;


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern volatile boot_struct * BootData;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Svc_RtcTask(void);
status_t GetCpuTemperature(float *temperature);
float GetCpuAverageTemperature(void);

void RTC_SetRTC_BKP_CRC(void);

#endif    /* SVC_RTC_H */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
