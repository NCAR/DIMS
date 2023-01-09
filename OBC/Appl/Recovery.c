#include "Logging.h"
#include "defs.h"
#include "Recovery.h"
#include "RunTime.h"
/***************************************
 * This files helps with the Recovery of the Different HAL systems of the OBC
 * Author: mjeffers@ucar.edu
*/

/*************************
 * @brief: This will reset the I2C and SPI ports
*/
void Recovery_HAL_Reset(void)
{
    print("Reinit I2C\n\r");
    HAL_StatusTypeDef ret;
    HAL_I2C_DeInit(&hi2c3);         //Release IO port as GPIO, reset handle status flag
    osDelay(500);
    ret = HAL_I2C_Init(&hi2c3);

    print("Reinit SPI\n\r");
    HAL_SPI_DeInit(&hspi1);         //Release IO port as GPIO, reset handle status flag
    osDelay(500);
    ret = HAL_SPI_Init(&hspi1);
    osDelay(500);
}

/**
  * @brief  Launches recovery based on HAL Issues if too many attempts this will launch a reset
  * @param  ret : The HAL_StatusTypeDef notrmal return type of the HAL System
  * @param  isI2C : 1 if I2C, 0 if SPI
  * @param attempts : how amy times we have tried to recover
  * @retval none
  */
void HAL_Recovery_Tree(HAL_StatusTypeDef ret, bool isI2C, uint8_t attempts){
   //Woring This Can Currently Turn into An inifinite Loop, need to becareful with Recusion
   //Findout what error we are getting Error
   if((attempts > 5)&&(ret != HAL_OK)){
     print("HAL_Recovery_Tree: Too Many Attempts Need to Restart\r\n");
     Restart_System();
   }
   if(ret == HAL_OK){
       print("HAL Was able to Recover: returning to main program\r\n");
       return;
   }
   if(isI2C){
        HAL_I2C_Recovery_Tree(ret, attempts);
  //If we are not using I2C
   }else{
        HAL_SPI_Recovery_Tree(ret, attempts);
   }

   return;
}

/**
  * @brief  Launches recovery based on HAL Issues
  * @param  ret : The HAL_StatusTypeDef normal return type of the HAL System
  * @param attemps : how many times we have tried to recover
  * @note  This is a helper function for HAL_Recovery_Tree
  * @note  This is attempts to recovery in the Same way no matter the issue at this pont 
  */
void HAL_SPI_Recovery_Tree(HAL_StatusTypeDef ret, uint8_t attempts){
    if(attempts > 5){
      print("HAL_SPI_Recovery_Tree: Too Many Attempts Need to Restart\n\r");
      HAL_Recovery_Tree(ret, 0, attempts);
    }
    if(ret == HAL_ERROR){
        print("HAL SPI has an error\n\r");
        HAL_Recovery_State_Busy_SPI(attempts);
    }else if(ret == HAL_BUSY){
        print("HAL SPI Was unexpectedly Busy maybe stuck\n\r");
        HAL_Recovery_State_Busy_SPI(attempts);
    }else if(ret == HAL_TIMEOUT){
        print(" HAL SPI Timed Out\r\n");
        HAL_Recovery_State_Busy_SPI(attempts);
    }else if (ret == HAL_OK){
        print("HAL SPI Was Okay\n\r");
    }
}


/**
  * @brief  Launches recovery based on HAL Issues
  * @param  ret : The HAL_StatusTypeDef normal return type of the HAL System
  * @param attemps : how many times we have tried to recover
  * @retval none
  * @note  This is a helper function for HAL_Recovery_Tree
  * @note  This is attempts to recovery in the Same way no matter the issue at this pont 
  */
void HAL_I2C_Recovery_Tree(HAL_StatusTypeDef ret, uint8_t attempts){
    if(attempts > 5){
      print("HAL_I2C_Recovery_Tree: Too Many Attempts Need to Restart\n\r");
      HAL_Recovery_Tree(ret, 1, attempts);
    }
    if(ret == HAL_ERROR){
        print("HAL I2C has an error\n\r");
        HAL_Recovery_State_Busy_I2C(attempts);
    }else if(ret == HAL_BUSY){
        print("HAL I2C Was unexpectedly Busy maybe stuck\n\r");
        HAL_Recovery_State_Busy_I2C(attempts);
    }else if(ret == HAL_TIMEOUT){
        print(" HAL I2C Timed Out\r\n");
        HAL_Recovery_State_Busy_I2C(attempts);
    }else if (ret == HAL_OK){
        print("HAL I2C Was Okay\n\r");
        HAL_Recovery_Tree(ret, 1, attempts);
    }
}
/**
  * @brief  Launches recovery based on XCAM Issues Will toggle the I2C Bus then Jump back into the Recovery Tree
  * @retval none
  */
void HAL_Recovery_State_Busy_I2C(uint8_t attempts){
    print("Attempting I2C Toggle Recovery\n\r");
    HAL_StatusTypeDef ret;
    HAL_I2C_DeInit(&hi2c3);         //Release IO port as GPIO, reset handle status flag
    ret = HAL_I2C_Init(&hi2c3);
    attempts++;
    HAL_I2C_Recovery_Tree(ret, attempts);
}

/**
  * @brief  Launches recovery based on XCAM Issues Will toggle the SPI Bus then Jump back into the Recovery Tree
  * @retval none
  */
void HAL_Recovery_State_Busy_SPI(uint8_t attempts){
    print("Attempting I2C Toggle Recovery\n\r");
    HAL_StatusTypeDef ret;
    HAL_SPI_DeInit(&hspi1);         //Release IO port as GPIO, reset handle status flag
    ret = HAL_SPI_Init(&hspi1);
    attempts++;
    HAL_SPI_Recovery_Tree(ret, attempts);
}


/*******
 * @brief  Launches recovery based on XCAM Issues Will launch Different Recovery Attempts based on issue
 * @retval none
 */
void XCAM_Recovery_Tree(uint8_t Status){
//  main_imaging_loop(0);
  return;
}

/** ***
 * @brief  Writes to the Housekeeping File then restarts the System
 * @retval none
 */
void Restart_System(){
    print("Restarting System\n\r");
    //Unmount the SD Card
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
    //Reset the System
    //WARNING this Currently Doesnt Work, need to find a way to reset the system
    //I think the issue is with the boot loader not jumping to the right location
    //HAL_NVIC_SystemReset();

    //Alternative
//    main_imaging_loop(0);
}
