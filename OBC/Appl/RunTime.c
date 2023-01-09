#include "RunTime.h"
#include "defs.h"
#include "Recovery.h"

uint8_t Exposure = 0;


//Check the Voltage of the EPS Battery
//If the Battery Level is too low then the XCAM will be powered off to allow the EPS battery to recharge until it reaches a certain voltage level
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
    //If the EPS battery Voltage is less than 3.6V then the XCAM will be powered off to allow the EPS battery to recharge
    if(Voltage<3.6){
        EPS_check(1,1);
        D_XCAM_Power_Off();
        EPS_check(1,1);
        //While the Voltage is less than 3.9V the XCAM will wait for the EPS battery to recharge
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

//This Function will run the General Routine For the XCAM
// This Includes:
// 1. Powering off the XCAM ensuring it is in a known state
// 2. Setting the Exposure
// 3. Turing on the EPS fast Charge ability
// 4. Setting up the SD Card
// 5. Setting up the XCAM
// 6. Adjust the Exposure
// 7. Take the Picture
// 8. Check the Battery level
// 9. Repeat from step 6
void XCAM_Run()
{
    HAL_StatusTypeDef ret = HAL_ERROR;



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


    //Setup the SD Card
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

    //How many images have we taken
    uint8_t total_captures = 0;

    //While the total captures is less than 30
    while(++total_captures < 30)
    {

        //Check the Battery Level
        CheckVoltage();

        // set exposure time
        Adjust_Exposure(Exposures[Exposure]);

        //Attempt to set the Imaging Made three times
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
        //Exit if we couldnt set up the XCAM
        if (result != 0)
            return;

        //Write the Statuses To the HK and Ensure Everything is okay
        D_XCAM_GetStatus(D_XCAM_Status);
        D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

        //Begin the Exposure
        D_XCAM_BeginExposure(); // begin capture
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

        //Wait for the Exposure to complete as long as we dont go over 90 Seconds of waiting
        for (i=0; i<90; i++)
        {
            //Analyse the Status of the XCAM and break if the image is complete
            if (D_XCAM_AnalyzeStatus(D_XCAM_Status,&packetsRemaining, &Error_Flag) & 0x02)
            {
                print("Image Capture Complete Attempting to break Loop\r\n");
                break;
            }
            //Keep waiting for the image to complete
            print("Waiting for image...\r\n");
            osDelay(1000);    /* Give processing time for the other tasks */
            //If we arent able to Check the Status
            if (D_XCAM_GetStatus(D_XCAM_Status))
            {
                print("Error checking status.\r\n");
                osDelay(1000);
            }

            //Send an Alive signla to the OBC
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        }

        //Get the Exposure to a file
        print("Image Capture Complete\r\n");
        sprintf(buffer, "Writing Exposure: %i\r\n", Exposures[Exposure]);
        print(buffer);
        D_XCAM_SendInitOrUpdate(false, false);
        D_XCAM_GetEntireImageSPIFast();
        //Go to the Next exposure
        Exposure++;
        if (Exposure > 4)
          Exposure = 0;
        D_XCAM_SendInitOrUpdate(false, true);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    }
}

