/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file AppTasks.c
* @brief Implementation of common task primitives
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Vasil Milev
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
* @revision{         1.0.0  , 2018.07.04, author Vasil Milev, Initial revision }
* @revision{         1.0.1  , 2020.01.16, author Georgi Georgiev, Moved everything, except StartDefaultTask() to DefTasks.c }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "AppTasks.h"
#include "fatfs.h"
#include "ESTTC.h"
#include "TaskMonitor.h"
#include "User_types.h"
#include "LIS3MDL_MAG_driver.h"
#include "DAT_Inputs.h"
#include "Panels.h"
#include "version.h"
#include "S_Band_Trnsm.h"
#include "X_Band_Trnsm.h"
#include  <Svc_RTC.h>
#include  <RunTime.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define APP_LED_ON_TIME             (50)                                /* given time in ms */
#define APP_TASK_CALL_PERIOD        (1000)                              /* given time in ms */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief That is a task used as example to configure all sensors and actuators and blinks the green LED.
* That can be changed freely as needed depending on the project.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      argument - not used
* @param[output]     none
* @return            none
* @note              none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void StartDefaultTask(void const * argument)
{

  //-------------------- sensors init start and print to both communication channels ------------------
  // That is just an example how to initialise all sensors and to print to two USART channels if needed

      if (Magnitometers_LIS3MDL_Init(LIS3MDL_MAG_I2C_ADDRESS_LOW) == E_OK)
      {
          fprintf(PAYLOAD,"MAG1_INIT_OK\r");
          fprintf(COMM  ,"MAG1_INIT_OK\r");
      }
      else{
          fprintf(PAYLOAD, "  MAG1 fail\r");
          fprintf(COMM  , "  MAG1 fail\r");
      }
      if (Magnitometers_LIS3MDL_Init(LIS3MDL_MAG_I2C_ADDRESS_HIGH) == E_OK)
      {
          fprintf(PAYLOAD,"MAG2_INIT_OK\r");
          fprintf(COMM  ,"MAG2_INIT_OK\r");
      }
      else{
          fprintf(PAYLOAD, "  MAG2 fail\r");
          fprintf(COMM  , "  MAG2 fail\r");
      }

      //Inizialize ACC Sensor 1
      if (AIS328DQ_Init(AIS328DQ_1_MEMS_I2C_ADDRESS) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"ACC1_INIT_OK\r");
        fprintf(COMM  ,"ACC1_INIT_OK\r");
      }
      else{
        I2C_Reset(&hi2c2);
      }

      //Inizialize ACC Sensor 2
      if (AIS328DQ_Init(AIS328DQ_2_MEMS_I2C_ADDRESS) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"ACC2_INIT_OK\r");
        fprintf(COMM  ,"ACC2_INIT_OK\r");
      }
      else{
        I2C_Reset(&hi2c2);
      }

      Panels_Init();

      /* Set PWM of the magnetorquer to 0% (i.e. OFF) */
      if (SetMagnetorque(PAN_X_M, 0, 1) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"TRQ%u_INIT_OK 0%%\r", TRQ_1);
        fprintf(COMM  ,"TRQ%u_INIT_OK 0%%\r", TRQ_1);
      }

      /* Set PWM of the magnetorquer to 0% (i.e. OFF) */
      if (SetMagnetorque(PAN_Y_M, 0, 1) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"TRQ%u_INIT_OK 0%%\r", TRQ_2);
        fprintf(COMM  ,"TRQ%u_INIT_OK 0%%\r", TRQ_2);
      }

      /* Set PWM of the magnetorquer to 0% (i.e. OFF) */
      if (SetMagnetorque(PAN_Z_M, 0, 1) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"TRQ%u_INIT_OK 0%%\r", TRQ_3);
        fprintf(COMM  ,"TRQ%u_INIT_OK 0%%\r", TRQ_3);
      }

      //Inizialize GYR Sensor X
      if (ADIS16265_Init(PAN_X_M) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"GYR%u_INIT_OK\r", GYR_1);
        fprintf(COMM  ,"GYR%u_INIT_OK\r", GYR_1);
      }
      else{
        fprintf(PAYLOAD,"GYR%u_INIT_FAIL\r", GYR_1);
        fprintf(COMM  ,"GYR%u_INIT_FAIL\r", GYR_1);
      }

      //Inizialize GYR Sensor Y
      if (ADIS16265_Init(PAN_Y_M) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"GYR%u_INIT_OK\r", GYR_2);
        fprintf(COMM  ,"GYR%u_INIT_OK\r", GYR_2);
      }
      else{
        fprintf(PAYLOAD,"GYR%u_INIT_FAIL\r", GYR_2);
        fprintf(COMM  ,"GYR%u_INIT_FAIL\r", GYR_2);
      }

      //Inizialize GYR Sensor Z
      if (ADIS16265_Init(PAN_Z_M) == SEN_SUCCESS)
      {
        fprintf(PAYLOAD,"GYR%u_INIT_OK\r", GYR_3);
        fprintf(COMM  ,"GYR%u_INIT_OK\r", GYR_3);
      }
      else{
        fprintf(PAYLOAD,"GYR%u_INIT_FAIL\r", GYR_3);
        fprintf(COMM  ,"GYR%u_INIT_FAIL\r", GYR_3);
      }

      //----------------------------sensors init end
      TaskMonitor_TaskInitialized(TASK_MONITOR_DEFAULT);   /* The task is initialized and is ready */

      main_imaging_loop();   // Damon's nice code

      for( ; ; )
      {
          TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT); /* Prevent from WatchDog reset */

          /* blink the Green LED for 50ms to indicate the OBC is running */
          GREEN_LED_ON();
          osDelay(APP_LED_ON_TIME);
          GREEN_LED_OFF();

          osDelay(APP_TASK_CALL_PERIOD);    /* Give processing time for the other tasks */
      }

}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
