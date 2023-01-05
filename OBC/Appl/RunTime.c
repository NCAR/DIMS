#include "RunTime.h"
#include "defs.h"
#include "Recovery.h"

uint8_t Exposure = 0;

void CheckVoltage()
{
    float Voltage = 0;
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    EPS_getBattery_voltage(&Voltage);
    //Check the Battery Level it its low take a break
    char buffer[100];
    EPS_check(1,1);
    sprintf(buffer, "Current EPS Voltage: %.3f\n\r", Voltage);
    print(buffer);
    if(Voltage<3.6){
        EPS_check(1,1);
        D_XCAM_Power_Off();
        EPS_check(1,1);
        while(Voltage<3.9){
            print("Waiting For Battery to recharge.\r\n");
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
            osDelay(10000);
            sprintf(buffer, "Current EPS Voltage: %.3f\n\r", Voltage);
            print(buffer);
            EPS_check(1,1);
            EPS_getBattery_voltage(&Voltage);
        }
    }
}


void XCAM_Run()
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t gps[34] = {0};



    D_XCAM_Power_Off(); // do this right away so batteries can charge
    bool Error_Flag = false;
    uint8_t i, result;
    //Setup The Registers
    uint8_t D_XCAM_Status[22] = {0};
    uint16_t packetsRemaining = 0;
    //Exposure Settings
    uint8_t len = 5;
    uint16_t Exposures[5]= {0};
    Exposures[0] = 0;//Set Exposure to Auto
    Exposures[1] = 80; //in units of 63uS
    Exposures[2] = 159;
    Exposures[3] = 238;
    Exposures[4] = 317;
    //Turn on Fast Charge on the EPS
    EPS_write(9, 1);
    EPS_write(8, 1);
    EPS_check(1,1);


    //Setup the SD Card <-- no you don't rename this thing
    if(Setup_SD()){
        print("Couldn't setup SD\r\n");
        //TO-DO Probably need to restart SD
    }
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    //Initalize the XCAM
    osDelay(1000);
    if(D_XCAM_Initialize_XCAM()){
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        //Initialization Failure
        print("Couldn't initialize XCAM\r\n");
        print("Restarting XCAM and Main Loop\r\n");
        Recovery_HAL_Reset();
        return;
    }

    uint8_t total_captures = 0;

    CheckVoltage();


//Check-out Curly Bracket issues here
    while(++total_captures < 30)
    {
        ret = HAL_I2C_Master_Receive(&hi2c3, 70 << 1,
                                     gps, 32, 100);
        fprintf(PAYLOAD, "GPS %s\r\n", gps);
        osDelay(9);
        CheckVoltage();
        EPS_check(1,1);
        // set exposure time
        Adjust_Exposure(Exposures[Exposure]);
        for (i=0; i<4; i++)
        {
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
            result = D_XCAM_EnableImagingMode();
            if (result == 0)
                break;
            print("Couldn't set imaging mode\r\n");
            osDelay(1000);
            print("Trying again\r\n");
        }
        if (result != 0)
            return;
        //Write the Statuses To the HK and Ensure Everything is okay
        D_XCAM_GetStatus(D_XCAM_Status);
        D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);


        D_XCAM_BeginExposure(); // begin capture
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

        for (i=0; i<90; i++)
        {
            if (D_XCAM_AnalyzeStatus(D_XCAM_Status,&packetsRemaining, &Error_Flag) & 0x02)
            {
                print("Image Capture Complete Attempting to break Loop\r\n");
                break;
            }
            print("Waiting for image...\r\n");
            osDelay(1000);    /* Give processing time for the other tasks */
            if (D_XCAM_GetStatus(D_XCAM_Status))
            {
                print("Error checking status.\r\n");
                osDelay(1000);
            }
//           D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        }

        //Get the Exposure to a file
        print("Image Capture Complete\r\n");
        char buffer[100];
        sprintf(buffer, "Writing Exposure: %i\r\n", Exposures[Exposure]);
        print(buffer);
        D_XCAM_SendInitOrUpdate(false, false);
        D_XCAM_GetEntireImageSPIFast();
        Exposure++;
        if (Exposure > 4)
          Exposure = 0;
        D_XCAM_SendInitOrUpdate(false, true);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    }
}


