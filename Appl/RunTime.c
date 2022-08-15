#include "RunTime.h"
#include "defs.h"
#include "Recovery.h"


/******
 * @brief  Main function To Orchestrate all the Imaging functions
 * @param  tries Represents how many Unsuccessful attempts have been made to rerun the Imaging Loop
 * @retval None
 */
void main_imaging_loop(uint8_t tries){
    uint8_t max_tries = 4;
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    //If we have tried hings too many Times its Time to Restart The System

    if(tries > max_tries){
        Restart_System();
    }

    //Turn on Fast Charge on the EPS
    EPS_write(9, 1);
    EPS_write(8, 1);
    EPS_check(1,1);
    osDelay(10);
    float Voltage;
    EPS_getBattery_voltage(&Voltage);
    //Check the Battery Level it its low take a break
    if(Voltage<3.70){
        D_XCAM_Power_Off();
        while(Voltage<4.0){
            print("Waiting For Battery to recharge");
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
            osDelay(20000);
            EPS_check(1,1);
            EPS_getBattery_voltage(&Voltage);
        }
        main_imaging_loop(0);
    }

    //Setup The Registers
    uint8_t D_XCAM_Status[22] = {0};
    uint16_t packetsRemaining;
    
    //Exposure Settings
    uint8_t len = 5;
    uint16_t Exposures[5];
    Exposures[0] = 0;//Set Exposure to Auto
    Exposures[1] = 80; //in units of 63uS
    Exposures[2] = 159;
    Exposures[3] = 238;
    Exposures[4] = 317;
    
    //Setup the SD Card
    if(Setup_SD()){
        print("Couldn't setup SD\r\n");
        //TO-DO Probably need to restart SD
    }
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    
    //Initalize the XCAM
    if(D_XCAM_Initialize_XCAM()){
        //Initialization Failure
        print("Couldn't initialize XCAM\r\n");
        print("Restarting XCAM and Main Loop\r\n");
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        main_imaging_loop(tries++);
    }
    fprintf(PAYLOAD, "Wating after initializiation");
    osDelay(9);
    fprintf(PAYLOAD, "Done Waiting");

    uint8_t i = 0;// Keeps Track of Which Exposure we are on
    while(1){
        bool Error_Flag;
        //If the Index is too high we need to restart fom the beginning of the Exposure list
        //Check Battery Voltage
        float Voltage;
        EPS_getBattery_voltage(&Voltage);
        if(Voltage<3.70){
            D_XCAM_Power_Off();
            while(Voltage<4.0){
                print("Waiting For Battery to recharge");
                TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
                osDelay(20000);
                EPS_check(1,1);
                EPS_getBattery_voltage(&Voltage);
            }
            main_imaging_loop(0);
        }
        if(i > len){
            i = 0;
        }

        // set exposure time
        Adjust_Exposure(Exposures[i]);
        
        //Enable Imaging Mode on the XCAM
        uint8_t Enable_Imaging_Attempts = 4;
        uint8_t Current_Attempt = 0;
        while((D_XCAM_EnableImagingMode())&&(Current_Attempt < Enable_Imaging_Attempts)){
            print("Couldn't set imaging mode\r\n");
            print("Trying again\r\n");
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        }
        if(Current_Attempt == Enable_Imaging_Attempts){
            print("Couldn't set imaging mode\r\n");
            print("Restarting XCAM and Main Loop\r\n");
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
            main_imaging_loop(tries++);
        }

        //Write the Statuses To the HK and Ensure Everything is okay
        D_XCAM_GetStatus(D_XCAM_Status);
        D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

        D_XCAM_BeginExposure(); // begin capture
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        
        uint8_t counter = 0;
        uint8_t MaxTime = 90;
        //if e havent ran out of time and the Curent status is busy
        while((counter<=MaxTime)&&(!(D_XCAM_AnalyzeStatus(D_XCAM_Status,&packetsRemaining, &Error_Flag) & 0x02))){
            
            print("Waiting for image...\r\n");
            osDelay(1000);    /* Give processing time for the other tasks */
            
            D_XCAM_GetStatus(D_XCAM_Status);
            D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
            D_XCAM_GetStatus(D_XCAM_Status);
            D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
            D_XCAM_GetStatus(D_XCAM_Status);
            D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);

            //Send an Alive Signal
            if(counter%20==0){
                print("Sending Alive Signal\r\n");
                TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
            }

            counter = counter+1;

        }
        //If it took too long restart main loop
        if(counter>MaxTime){
            print("Image Capture Failed: Took Too long\r\n");
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
            main_imaging_loop(tries++);
        }

        
        //Get the Exposure to a file
        print("Image Capture Complete\r\n");
        char buffer[50];
        sprintf(buffer, "Writing Exposure: %i\r\n", Exposures[i]);
        print(buffer);
        D_XCAM_SendInitOrUpdate(false, false);
        D_XCAM_GetEntireImageSPI();
        D_XCAM_SendInitOrUpdate(false, true);

        // Write_Image_To_SD(PayloadI2C, 260);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

        //increment index for exposures
        i=i+1;

    }//End While Forever loop



}//End main imaging loop Function
