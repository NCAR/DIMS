/*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * EPS code alice@ucar.edu 2020 12 30
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
// This code queries the EPS battery voltage and current.
// It writes the information out to a file on the OBC as
// well as to the PAYLOAD terminal emulator. --Alice
#include "EPS.h"


void EPS_check(int printToFile, int printToPayload ) {


    long  battV, battC;
    long  voltZ;
    long  statusLUP3, statusLUP5;
    float battVconv = 2.346;
    float battCconv = 3.05 ;
    char battery_text[100];
    long Defaults;
    //float voltZconv = 2.4414063;
    char  timeStr[25];
    FIL  fidB;
    //UINT bw;
    FRESULT fresult;
    FILINFO  file_info;
    char filenm[] = "0:/BATT.txt";

    EPS_read( 1,&battV);  //  1 is battery voltage
    EPS_read( 2,&battC);  //  2 is battery current
    EPS_read(11,&voltZ);  // 11 is panel Z voltage
    EPS_read(43,&Defaults);
    fprintf(PAYLOAD, "Default Value %i\r\n", Defaults);
    EPS_read(16,&statusLUP3);
    EPS_read(17,&statusLUP5);
    HAL_RTC_GetTime(&hrtc,&sTime,calendar_format);  // must be before GetDate
    HAL_RTC_GetDate(&hrtc,&sDate,calendar_format);
    sprintf(timeStr,"%04d-%02d-%02d %02d:%02d:%02d",
        sDate.Year,sDate.Month,sDate.Date,sTime.Hours,sTime.Minutes,sTime.Seconds);

    sprintf(battery_text,
        "%s EPS Batt: voltage(mV): %ld %.2f  \tcurrent(mA): %ld %.2f\r",
        timeStr,
        battV, battVconv*(float)battV,
        battC, battCconv*(float)battC);


    if(printToFile==1) {
        fresult = f_open(&fidB, filenm,
                         FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
                      // FA_WRITE|FA_READ|FA_OPEN_ALWAYS|FA_OPEN_EXISTING);//,FA_OPEN_APPEND);
        if( fresult == FR_OK ) {
            fprintf(PAYLOAD,"****************BATT.txt SUCCESS!!!!!!!\r\n");
            fresult = f_stat (filenm, &file_info);
            if( fresult == FR_OK ) {
                fprintf(PAYLOAD,"****************BATT.txt f_stat SUCCESS!!!!!!!\r\n");
                fresult = f_lseek(&fidB, file_info.fsize);   //Go to the end of the file
                if( fresult == FR_OK ) {
                    fprintf(PAYLOAD,"****************BATT.txt f_lseek SUCCESS!!!!!!!\r\n");
                    fprintf((FILE *)&fidB,battery_text);
                    //f_write(&fidB, battery_text, sizeof(battery_text) , &bw);
                }
            }
            f_close(&fidB);
        }
        else {
            fprintf(PAYLOAD,"****************BATT.txt could not be opened!\r\n");
            fprintf(PAYLOAD,"****************ERROR(%u)\r\n", (uint16_t)fresult);
        }
    }
    if(printToPayload==1) {
        /*
        fprintf(PAYLOAD,battery_text);
        fprintf(PAYLOAD,
            "EPS Z panel: voltage(mV): %ld %.2f\r",
            voltZ, voltZconv*(float)voltZ);
        */
        fprintf(PAYLOAD,
            "EPS status LUP3 LUP5: %ld %ld \r\n", statusLUP3, statusLUP5 );
    }
}
bool EPS_Voltage_Level_Low(float lower_bound){

    float Voltage;
    EPS_getBattery_voltage(&Voltage);
    if(Voltage<lower_bound){
        return true;
    }else{
        return false;
    }
}

//Get the Battery Voltage:
void EPS_getBattery_voltage(float * PtrVoltage){
    long BatteryV;
    float BV;
    EPS_read(1, &BatteryV);
    BV = (float)BatteryV*.0023394775;
    char buffer[20];
    //strcat(buffer,);
    //print(buffer);
    *PtrVoltage = BV;
}

// This routine will reads from I2C1 which is the EPS. --Alice
void EPS_read(uint16_t Cmd, long *pEpsValue) {
    HAL_StatusTypeDef I2C_retStat;
    uint8_t timeout = 0;
    uint8_t temp_reg_buff[2];
    uint16_t eps_reg;

    *pEpsValue = -9999;  // initialize to a bad value

    /* Read to Communication interface I2C */
    MX_I2C1_Init(); //Enable I2C1 interface
    do {
        if( timeout >= 1 ) {
            // if there is any error reset the I2C interface
            if((  I2C_retStat == HAL_ERROR )
             ||(  I2C_retStat == HAL_TIMEOUT)
             ||(( I2C_retStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY))
             )
            {
                I2C_Reset(&hi2c1);
            }
            osDelay(5);

            if( timeout >= 10 ) { // Stop trying after certain times
                break;
            }
        }

        //Read the register from the EPS
        //HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c,
        //                 uint16_t DevAddress,
        //                 uint16_t MemAddress,
        //                 uint16_t MemAddSize,
        //                 uint8_t *pData,
        //                 uint16_t Size,
        //                 uint32_t Timeout);
        I2C_retStat = HAL_I2C_Mem_Read(
                          &hi2c1,               // I2C_HandleTypeDef
                          EPS_I2C_ADDRESS<<1,   // DevAddress
                          Cmd,                  // MemAddress
                          sizeof(uint8_t),      // MemAddSize
                          temp_reg_buff,        // *pData
                          sizeof(uint16_t),     // Size
                          10);                  // Timeout
        timeout ++; //count one more try
    }while( I2C_retStat != HAL_OK);

    HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface

    if (HAL_OK == I2C_retStat) {
        //the register is read successfully
        eps_reg = (temp_reg_buff[0] << 8) + temp_reg_buff[1];
        *pEpsValue = (long)eps_reg;

        //Print out the register value
        //fprintf(PAYLOAD,
        //    "EPS OK+0x%04X   %04ld \t", eps_reg,(long int)eps_reg);
    }
    else{
        //Reading has failed. Possibly the EPS is missing
        fprintf(PAYLOAD,"EPS ERR - executing");
    }

}


// This routine will writes to I2C1 which is the EPS. --Alice
void EPS_write(uint16_t memAddress, uint8_t Value) {
    HAL_StatusTypeDef I2C_retStat;
    uint8_t timeout = 0;

    uint8_t *pData, transmData[2];
    pData = &transmData[0];


    /* Read to Communication interface I2C */
    MX_I2C1_Init(); //Enable I2C1 interface
    do {
        if( timeout >= 1 ) {
            // if there is any error reset the I2C interface
            if((  I2C_retStat == HAL_ERROR )
             ||(  I2C_retStat == HAL_TIMEOUT)
             ||(( I2C_retStat == HAL_BUSY)&&(hi2c1.State == HAL_I2C_STATE_READY))
             )
            {
                I2C_Reset(&hi2c1);
            }
            osDelay(5);

            if( timeout >= 10 ) { // Stop trying after certain times
                break;
            }
        }

        // HAL_StatusTypeDef
        //     HAL_I2C_Mem_Write(
        //     I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
        //     uint16_t MemAddress,     uint16_t MemAddSize,
        //     uint8_t *pData,          uint16_t Size,         uint32_t Timeout);
        /*
        transmData[0] = Value;
        I2C_retStat = HAL_I2C_Mem_Write(&hi2c1, EPS_I2C_ADDRESS<<1,
                      memAddress,      sizeof(uint16_t),
                      &transmData[0],  sizeof(uint8_t), 10);
        */

        //HAL_StatusTypeDef HAL_I2C_Master_Transmit(
        //    I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
        //    uint8_t *pData,          uint16_t Size, uint32_t Timeout);
        /**/
        transmData[0] = (uint8_t)memAddress;
        transmData[1] = Value;
        I2C_retStat = HAL_I2C_Master_Transmit(&hi2c1, EPS_I2C_ADDRESS<<1,
            pData, 2, 10);
        /**/

        timeout ++; //count one more try
    }while( I2C_retStat != HAL_OK);

    HAL_I2C_DeInit(&hi2c1); //Disable I2C1 interface

    if (HAL_OK == I2C_retStat) {
        //the register is read successfully
        //fprintf(PAYLOAD, "EPS OK");
    }
    else{
        //Reading has failed. Possibly the EPS is missing
        fprintf(PAYLOAD,"EPS ERR - executing\n");
    }

}
