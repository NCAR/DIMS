#include "RunTime.h"
uint16_t fileIterator = 0;

/******************************************************************************
 * @brief : This Will Run the our Desired Initialization of the XCAM
 * @retval: (1) if it fails to Initialize 
 *          (0) if it successfully initializes
 ****************************************************************************/
uint8_t Initialize_XCAM(void){
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);


    // set the SPI nCS pin high (disable)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    
    D_XCAM_WaitSeconds(2, true);  // wait for the other tasks to stop printing text
    D_XCAM_Power_Cycle();
    
    uint8_t max_tries = 3;
    uint8_t tries = 0;
    
    while((D_XCAM_Init())&&(tries < max_tries)){
        #ifdef DEBUG
            fprintf(PAYLOAD, "Couldn't initialize XCAM\r\n");
            fprintf(PAYLOAD, "Attempting to Initialize Again\r\n");
        #endif
        Write_To_HK("Couldn't initialize XCAM\r\n");
        Write_To_HK("Attempting to Initialize again\r\n");
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        tries++;
    }
    //If we ran out of tries to Initialize
    if(tries == max_tries){
        #ifdef DEBUG
            fprintf(PAYLOAD, "Couldn't initialize XCAM\r\n");
        #endif
        Write_To_HK("Couldn't initialize XCAM\r\n");
        return 1;
    }
    
    //  4) Write to parameter 0x00 to identify which interface you wish to acquire an image from (0 for SI0, 1 for SI1, 2 for SI2).
    D_XCAM_SetParameter(0x00, 1);
    Write_To_HK("Set Param 0x00 to 1\r\n");
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

    //  5) Write to parameter 0x10 the file address where you wish the images to be stored. This will be included in each packet as an unsigned short in bytes 6 and 7.
    D_XCAM_SetParameter(0x10, 0);
    Write_To_HK("Set Param 0x10 to 0\r\n");
    return 0;

}

/****************************************************************
 * @brief : This will Create a header File for A given Image From the XCAM. Info Should include Time, Exposure Time, Parameter Output, XCAM Status, and Image Size.
 * @param : FileName : The Name of the File to Create
 * @param : Exposure :
 * @retval: (1) if it fails to create the header file
 *          (0) if it successfully creates the header file
 *****************************************************************/
uint8_t Make_ImageHeader(const char *filename){
    SD_Make_File(filename);
    

    //Write the Parameter Output in the headerFile
    char ParameterOutput[10];
    sprintf(ParameterOutput, "Image Number: %d\r\n", D_XCAM_GetParameter(0x00));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Integration Time: %d\r\n", D_XCAM_GetParameter(0x01));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Grab Command: %d\r\n", D_XCAM_GetParameter(0x02));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Auto expose flag: %d\r\n", D_XCAM_GetParameter(0x03));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Healthcheck Status: %d\r\n", D_XCAM_GetParameter(0x04));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Compression Flag: %d\r\n", D_XCAM_GetParameter(0x07));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Thumbnail Flag: %d\r\n", D_XCAM_GetParameter(0x08));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Windowing Flag: %d\r\n", D_XCAM_GetParameter(0x0B));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window X-Start: %d\r\n", D_XCAM_GetParameter(0x0C));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window X-Stop: %d\r\n", D_XCAM_GetParameter(0x0D));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window Y-Start: %d\r\n", D_XCAM_GetParameter(0x0E));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window Y-Stop: %d\r\n", D_XCAM_GetParameter(0x0F));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window X-Stop: %d\r\n", D_XCAM_GetParameter(0x0D));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "File Address: %d\r\n", D_XCAM_GetParameter(0x10));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Thumbnail Compression Flag: %d\r\n", D_XCAM_GetParameter(0x11));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Grab wait timeout: %d\r\n", D_XCAM_GetParameter(0x12));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Integration time fractional part: %d\r\n", D_XCAM_GetParameter(0x13));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Ops Error Code: %d\r\n", D_XCAM_GetParameter(0x15));
    SD_Append_String_File(filename, ParameterOutput, strlen(ParameterOutput));
    return 0;

}


/****************************************************************************
 * @brief : This will pull The Entire Image Until the priority packets read 0 from the XCAM
 * @param buffer : Pointer to the buffer
 * @return (1) if it did not transmit (2) issue with HAL (3) if it did not receive (0)everything Went Well
 ****************************************************************************/
uint8_t D_XCAM_GetEntireImageI2C(uint8_t *buffer){
    uint8_t ImagePacket[260] = {0};
    uint8_t status[22] = {0};
    uint16_t packetsRemaining;
    D_XCAM_GetStatus(status);
    D_XCAM_AnalyzeStatus(status, &packetsRemaining);
    uint16_t i = 0;

    //Dumb Way To do this But Will work For now
    const char Image_FileName[10];
    const char Header_FileName[10];
    sprintf(Image_FileName, "%05d.raw", fileIterator);
    sprintf(Header_FileName, "%05d.txt", fileIterator);

    //const char Image_FileName[10];
    //const char Header_FileName[10];

    //get_next_image_id(&Image_FileName[0], &Header_FileName[0]);

    //get_next_image_id(Image_FileName, Header_FileName);
    //Image_FileName =
    SD_Make_File(Image_FileName);
    SD_Make_File(Header_FileName);
    
    //fresult = f_open(&fid, Image_FileName,FA_WRITE|FA_READ|FA_OPEN_ALWAYS|FA_OPEN_EXISTING);

    while(packetsRemaining>0){
        D_XCAM_GetImageSPI(&ImagePacket[0]);

        //f_write(&fid,(const void *)(&XCAM_partial_data[0]),btw , &bw);
        SD_Append_Data_File(Image_FileName, ImagePacket, sizeof(ImagePacket));
        D_XCAM_GetStatus(status);
        D_XCAM_AnalyzeStatus(status, &packetsRemaining);
        SD_Append_String_File(Header_FileName, status, sizeof(status));


        //print(("%d\n\r" , packetsRemaining));
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    }

}

void main_imaging_loop(void){
    //Setup The Registers
    uint8_t D_XCAM_Status[22] = {0};
    uint8_t PayloadSPI[260] = {0};
    uint8_t PayloadI2C[260] = {0};
    uint16_t packetsRemaining;
    
    //Exposure Settings
    uint8_t len = 4;
    uint16_t Exposures[4];
    Exposures[0] = 0;//Set Exposure to Auto
    Exposures[1] = 1; //in units of 63uS
    Exposures[2] = 30;
    Exposures[3] = 4000;

    uint8_t i = 0;
    
    
    //Setu the SD Card
    if(Setup_SD()){
        //if DEBUG is defined print message to terminal
        print("Couldn't setup SD\r\n");
        //Probably need to restart SD
    }
    
    //Initalize the XCAM
    if(Initialize_XCAM()){
        //if DEBUG is defined print message to terminal
        print("Couldn't initialize XCAM\r\n");
        print("Restarting XCAM and Main Loop\r\n");
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        main_imaging_loop();
    }

    while(1){
        //If the Index is too high we need to restart fom the beginning
        if(i > len){
            i = 0;
        }
    // set exposure time
        Adjust_Exposure(Exposures[i]);
        fprintf(PAYLOAD,"Adjusted Exposure to %i\r\n", Exposures[i]);


        //  6) Update the payload operation mode to 0x01 for imaging operations.
        //TODO Will Probably Need to Atempt again if there is an issue here
        //Then power Cycle if issue not resolved
        if(D_XCAM_EnableImagingMode()){
            print("Couldn't set imaging mode\r\n");
            TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        }

    //  7) A bitwise payload status flag of 0x01 indicates the payload is currently busy in the operation cycle.
        //TODO Print the Status to HK
        D_XCAM_GetStatus(D_XCAM_Status);
        D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining);
        print("Reading Payload Status\r\n");
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

    //  8) Write to parameter 0x02 a value of 0x01 to initiate image capture. Image capture takes approximately 1s. If the grab command is not received within the timeout, C3D will register a mode failure.
        D_XCAM_BeginExposure(); // begin capture
        
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        
        uint8_t counter = 0;
        uint8_t MaxTime = 90;
        while((counter<=MaxTime)&&(!(D_XCAM_AnalyzeStatus(D_XCAM_Status,&packetsRemaining) & 0x02))){
            
            print("Waiting for image...\r\n");
            
            D_XCAM_WaitSeconds(1, true);
            //  9) A payload status flag of 0x02 indicates the image capture is complete and the data packets have been successfully compiled. The payload will then return to standby mode 0x00.
            // 10) If the payload status flag reads bitwise 0x10 then the operation has failed for some reason (refer to section 6.5.3 for details). The payload will attempt to complete each operation three times before returning this code.
            D_XCAM_GetStatus(D_XCAM_Status);
            D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining);

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
            Main_Camera_Loop();
        }
        print("Image Capture Complete\r\n");

    // 11) The payload data packets waiting will be incremented as the payload returns to standby, to reflect the image packets waiting in the payload memory.
    // 12) Provided the default parameters are still loaded, the data packets waiting will contain an uncompressed thumbnail image and a compressed, unwindowed full image.
    // 13) Data can now be downloaded by the platform. Both I2C download commands will be treated identically by the payload.

        char buffer[50];
        sprintf(buffer, "Writing Exposure: %d", Exposures[i]);
        print(buffer);
        D_XCAM_GetEntireImageI2C(PayloadI2C);
        fileIterator++;

       // Write_Image_To_SD(PayloadI2C, 260);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        
    // 14) If the payload status flag reads bitwise 0x08 during a data transfer then no more packets are waiting in memory. The payload will then return to standby mode 0x00.

        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

        //increment index for exposures
        i=i+1;
    }//End While Forever loop



}
