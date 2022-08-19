/*
Damon's Reference XCAM Implementation
2021-08-24
*/
#include "D_XCAM.h"
#include "TaskMonitor.h"
#include "EPS.h"
#include "defs.h"
#include "SD_handler.h"
#include "Logging.h"
#include <stdbool.h>
#define MIN(a,b) (((a)<(b))?(a):(b))

#define PACKETGROUPSIZE (200)

uint8_t ImageBuffer[260*PACKETGROUPSIZE] = {0};//Buffer For 16 packets

#define D_XCAM_ADDRESS (0x66)
#define D_XCAM_DEBUG (1)
#define D_XCAM_HALOK (0)
#define D_XCAM_RAWSTATUS (1)
#define XCAM_BUSY (0x01)


//XCAM_Registers
#define XCAM_SENSOR        (0)   // use 1, WFI at HAO
#define XCAM_INTTIME       (1)
#define XCAM_GRAB          (2)
#define XCAM_AUTOEXP       (3)
#define XCAM_HEALTH        (4)
#define XCAM_COMPRESS      (7)
#define XCAM_THUMB         (8)
#define XCAM_WINDOW       (11)   // 0x0B
#define XCAM_TIMEOUT      (18)   // 0x12
#define XCAM_INTTIMEFRAC  (19)   // 0x13
#define XCAM_OP_ERROR     (21)   // 0x15


uint16_t XCAM_sensor   =  1; // 1 == WFI
uint16_t XCAM_autoexp  =  0; // 1 == use auto exposure
uint16_t XCAM_timeout  = 30; // the time C3D goes back to standby
                             // after a grab command
uint16_t XCAM_inttime  =  200;
// defined in Middlewares/ESTTC.h
// extern volatile uint16_t XCAM_inttime_desired;
uint16_t XCAM_inttimefrac = 0;  // just an adhoc value for now
uint16_t XCAM_window   =  0;
uint16_t XCAM_compress =  0; // = 0 no compression, 1=compression
uint16_t XCAM_thumb    =  0; // = 0 no thumbnail  , 1=thumbnail
uint16_t XCAM_grab     =  1; // = 0 no grab, 1=grab an image
uint8_t  XCAM_mode     =  1; // 1=imaging mode

/* Global Vars */
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;
#include <time.h>


/****************************************************************************
 * @brief : This will pull The Entire Image Until the priority packets read 0 from the XCAM
 * @param buffer : Pointer to the buffer
 * @return 1 if an error happened, 0 if everything went well
 ****************************************************************************/
uint8_t D_XCAM_GetEntireImageSPI(){
    uint8_t ImagePacket[260] = {0};//Buffer For Image Packet
    uint8_t status[22] = {0};//Buffer For Status of the Xcam
    uint16_t packetsRemaining;//Howmany Packets do We have left to Go through
    uint16_t num_download_requests = 0;//Keep Track Of our Downlaod Requests so we dont get stuck unexpectedly
    bool Error_Flag = false;//Flag to indicate if there was an error
    //Find out how many Packets need to be downloaded
    D_XCAM_GetStatus(status);
    D_XCAM_AnalyzeStatus(status, &packetsRemaining, &Error_Flag);
    if(Error_Flag == true){
        return 1;
    }
    uint16_t max_download_requests = packetsRemaining*1.5;// arbitary Value Assumes half of the packets are going to fail  

    //Find the Next available Filename
    char Image_FileName[10];
    char Header_FileName[10];

    get_next_image_id(&Image_FileName[0], &Header_FileName[0]);

    SD_Make_File(Image_FileName);
    SD_Make_File(Header_FileName);
    char buffer[50];
    sprintf(buffer, "Writing to: %s\r\n", Image_FileName);
    print(buffer);
    
    //Keep requewsting packets until we have them all or until its getting Absurd
    //This Take Forever mostly BC I am writing every file to buffer individually
    while((packetsRemaining>0)&&(num_download_requests<max_download_requests)){
      D_XCAM_GetImageSPI(&ImagePacket[0]);
      TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
      SD_Append_Data_File(Image_FileName, ImagePacket, sizeof(ImagePacket));
      D_XCAM_GetStatus(status);

      //Write to the Header File for the image
      D_XCAM_AnalyzeStatus(status, &packetsRemaining, &Error_Flag);
      if(Error_Flag == true){
        print("There was an error when processing the IMage\r\n");
        return 1;
      }
      
      //Every 30 Packets Send an alive Signal
      if(num_download_requests%500==0){
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        EPS_check(1,1);
      }

      num_download_requests++;
    }
  return 0;
}











uint8_t D_XCAM_GetEntireImageSPIFast(){
    uint16_t j;
    uint16_t i;
    uint8_t status[22] = {0};//Buffer For Status of the Xcam
    uint16_t packetsRemaining;//Howmany Packets do We have left to Go through
    bool Error_Flag = false;//Flag to indicate if there was an error
    //Find out how many Packets need to be downloaded
    D_XCAM_GetStatus(status);
    D_XCAM_AnalyzeStatus(status, &packetsRemaining, &Error_Flag);
    if(Error_Flag == true){
        return 1;
    }

    //Find the Next available Filename
    char Image_FileName[10];
    char Header_FileName[10];
    uint16_t PacketsToRequest = 0;
    get_next_image_id(&Image_FileName[0], &Header_FileName[0]);

    SD_Make_File(Image_FileName);
    SD_Make_File(Header_FileName);
    char buffer[50];
    sprintf(buffer, "Writing to: %s\r\n", Image_FileName);
    print(buffer);

    for (j=0; j<(5264/PACKETGROUPSIZE+10); j++)
    {
        D_XCAM_GetStatus(status);
        D_XCAM_AnalyzeStatus(status, &packetsRemaining, &Error_Flag);
        fprintf(PAYLOAD, "%d packets remain.\r\n",packetsRemaining);
        if (packetsRemaining == 0)
            break;
        PacketsToRequest = MIN(PACKETGROUPSIZE,packetsRemaining); // don't ask for more than are available
        for (i=0;i<PacketsToRequest; i++)
            D_XCAM_GetImageSPI(&ImageBuffer[i*260]);
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
//        SD_Append_Data_File(Image_FileName, ImageBuffer, sizeof(ImageBuffer));
        SD_Append_Data_File(Image_FileName, ImageBuffer, 260*PacketsToRequest);

    }
  return 0;
}













/******************************************************************************
 * @brief : This Will Run the our Desired Initialization of the XCAM
 * @retval: (1) if it fails to Initialize 
 *          (0) if it successfully initializes
 ****************************************************************************/
uint8_t D_XCAM_Initialize_XCAM(void){
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

    // set the SPI nCS pin high (disable)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
//    D_XCAM_WaitSeconds(2, true);  // wait for the other tasks to stop printing text
    //D_XCAM_Power_Cycle();
    D_XCAM_Power_On();
    CheckVoltage();
    uint8_t tries = 0;
    uint8_t result = 0;
    for (tries=0; tries<4; tries++)
    {
        TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
        result = D_XCAM_Init();
        if (result==0)
            break;
        print("Couldn't initialize XCAM\r\n");
        print("Attempting to Initialize again\r\n");
        osDelay(500);
    }
    //If we ran out of tries to Initialize
    if (result != 0){
        print("Couldn't initialize XCAM\r\n");
        return 1;
    }
    
    //  4) Write to parameter 0x00 to identify which interface you wish to acquire an image from (0 for SI0, 1 for SI1, 2 for SI2).
    
    D_XCAM_SetParameter(XCAM_SENSOR, 1);
    print("Set Param 0x00 to 1\r\n");
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

    //  5) Write to parameter 0x10 the file address where you wish the images to be stored. This will be included in each packet as an unsigned short in bytes 6 and 7.
    D_XCAM_SetParameter(0x10, 0);
    print("Set Param 0x10 to 0\r\n");
    return 0;
}

void D_XCAM_Write_EPS_8_9_bit1(void){
  EPS_write(9, 1);
  EPS_write(8, 1);
}


/****************************************************************
 * @brief : This will Create a header File for A given Image From the XCAM. Info Should include Time, Exposure Time, Parameter Output, XCAM Status, and Image Size.
 * @param : FileName : The Name of the File to Create
 * @param : Exposure :
 * @retval: (1) if it fails to create the header file
 *          (0) if it successfully creates the header file
 *****************************************************************/
uint8_t D_XCAM_Make_ImageHeader(char *filename){
    
/*
    //Write the Parameter Output in the headerFile
    char ParameterOutput[30];
    sprintf(ParameterOutput, "Image Number: %s\r\n", D_XCAM_GetParameter(0x00));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Integration Time: %s\r\n", D_XCAM_GetParameter(0x01));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Grab Command: %s\r\n", D_XCAM_GetParameter(0x02));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Auto expose flag: %s\r\n", D_XCAM_GetParameter(0x03));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Healthcheck Status: %s\r\n", D_XCAM_GetParameter(0x04));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Compression Flag: %s\r\n", D_XCAM_GetParameter(0x07));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Thumbnail Flag: %s\r\n", D_XCAM_GetParameter(0x08));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Windowing Flag: %s\r\n", D_XCAM_GetParameter(0x0B));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window X-Start: %s\r\n", D_XCAM_GetParameter(0x0C));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window X-Stop: %s\r\n", D_XCAM_GetParameter(0x0D));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window Y-Start: %s\r\n", D_XCAM_GetParameter(0x0E));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window Y-Stop: %s\r\n", D_XCAM_GetParameter(0x0F));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Window X-Stop: %s\r\n", D_XCAM_GetParameter(0x0D));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "File Address: %s\r\n", D_XCAM_GetParameter(0x10));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Thumbnail Compression Flag: %s\r\n", D_XCAM_GetParameter(0x11));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Grab wait timeout: %s\r\n", D_XCAM_GetParameter(0x12));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Integration time fractional part: %s\r\n", D_XCAM_GetParameter(0x13));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
    
    sprintf(ParameterOutput, "Ops Error Code: %s\r\n", D_XCAM_GetParameter(0x15));
    SD_Append_String_File(filename, &ParameterOutput[0], strlen(ParameterOutput));
*/
    return 0;

}


/*******************************************************************************
 * @brief  Begin Exposure by setting the Grab command to 1.
 * @param  None
 * @retval (0) if success, (1) if error
 * @note   This function is called by the main program.
 * ****************************************************************************/
uint8_t D_XCAM_BeginExposure(){
  if(D_XCAM_SetParameter(XCAM_GRAB, 1)){
    print("D_XCAM_SetParameter(XCAM_GRAB, 1) failed\r\n");
    return 1;
  }
  return 0;
}

/**
  * @brief  This should Adjust the Exposure Based on the given Setting
  * @param  setting : Duration of Exposure in units of 157us Note: if set to '0' the camera will use Auto Exposure
  * @retval none
  */
void Adjust_Exposure(uint8_t setting){
    char buffer[50];
    if(setting == 0){
        // set auto-exposure mode
        D_XCAM_SetParameter(XCAM_AUTOEXP, 1);
        print("Set to Auto Exposure\n\r");
    }else{
        //Set the Exposure Time
        D_XCAM_SetParameter(XCAM_INTTIME, setting);
        D_XCAM_SetParameter(XCAM_AUTOEXP, 0);
        sprintf(buffer, "Set Exposure time to: %d*63uS\r\n", setting);
        print(buffer);
    }        
    return;
}


void D_XCAM_Example(void){

  uint8_t D_XCAM_Status[22] = {0};
  uint8_t PayloadSPI[260] = {0};
  uint8_t PayloadI2C[260] = {0};
  uint16_t packetsRemaining;
  // set the SPI nCS pin high (disable)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  D_XCAM_WaitSeconds(2, true);  // wait for the other tasks to stop printing text
  D_XCAM_Power_On();
  
  D_XCAM_Init();

 // Write_To_File();
//  4) Write to parameter 0x00 to identify which interface you wish to acquire an image from (0 for SI0, 1 for SI1, 2 for SI2).
  D_XCAM_SetParameter(0x00, 1);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

//  5) Write to parameter 0x10 the file address where you wish the images to be stored. This will be included in each packet as an unsigned short in bytes 6 and 7.
  D_XCAM_SetParameter(0x10, 0);
  
// set exposure time
//  D_XCAM_SetParameter(0x01, 4000);

// set auto-exposure mode
  D_XCAM_SetParameter(0x03, 0x01);

//  6) Update the payload operation mode to 0x01 for imaging operations.
  D_XCAM_EnableImagingMode();

//  7) A bitwise payload status flag of 0x01 indicates the payload is currently busy in the operation cycle.
  D_XCAM_GetStatus(D_XCAM_Status);
  bool Error_Flag = false;
  D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

//  8) Write to parameter 0x02 a value of 0x01 to initiate image capture. Image capture takes approximately 1s. If the grab command is not received within the timeout, C3D will register a mode failure.
  D_XCAM_SetParameter(0x02, 0x01); // begin capture
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

  do{
    fprintf(PAYLOAD, "Waiting for image...\n\r");
    D_XCAM_WaitSeconds(1, true);
  //  9) A payload status flag of 0x02 indicates the image capture is complete and the data packets have been successfully compiled. The payload will then return to standby mode 0x00.
  // 10) If the payload status flag reads bitwise 0x10 then the operation has failed for some reason (refer to section 6.5.3 for details). The payload will attempt to complete each operation three times before returning this code.
    D_XCAM_GetStatus(D_XCAM_Status);
  }while (!(D_XCAM_AnalyzeStatus(D_XCAM_Status, &packetsRemaining, &Error_Flag) & 0x02));

  fprintf(PAYLOAD, "Image captured!\r\n");
// 11) The payload data packets waiting will be incremented as the payload returns to standby, to reflect the image packets waiting in the payload memory.
// 12) Provided the default parameters are still loaded, the data packets waiting will contain an uncompressed thumbnail image and a compressed, unwindowed full image.
// 13) Data can now be downloaded by the platform. Both I2C download commands will be treated identically by the payload.
  D_XCAM_GetImageI2C(PayloadI2C);
//  Write_Image_To_SD(PayloadI2C, 260);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  D_XCAM_GetImageSPI(PayloadSPI);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  
// 14) If the payload status flag reads bitwise 0x08 during a data transfer then no more packets are waiting in memory

  fprintf(PAYLOAD,"Damon's XCAM example is done. Halting.\n\r");
  
  while(1)
  {
    osDelay(100);
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  }
}




/**
 * @brief: This function Turns off the XCAM Device by first turning off the 5V then waiting 5 sec and Turning off the 3v3 bus
 * @param: none
 * @retvalue: none
 */ 
void D_XCAM_Power_Off(void){
  #ifdef DEBUG
  fprintf(PAYLOAD,"Turning off LUP 5v\r\n");
  #endif
  EPS_write(6,1);  //  --> turn off LUP 5v
//  D_XCAM_WaitSeconds(5, true);
  fprintf(PAYLOAD,"Turning off LUP 3v3\r\n");
  EPS_write(5,1);  //  --> turn off LUP 3.3v
  // wait for everything to settle
//  D_XCAM_WaitSeconds(5, true);
//  EPS_check(1,1);
  return;
}

/**
 * @brief: This function Turns on the XCAM Device by first turning on the 3v3 then waiting 5 sec and Turning on the 5 bus
 * @param: none
 * @retvalue: none
 */ 
void D_XCAM_Power_On(void){
  #ifdef DEBUG
  fprintf(PAYLOAD,"Turning on LUP 3v3\n\r");
  #endif
  EPS_write(5,0);  //  --> turn on LUP 3.3v
  // wait for everything to settle
  D_XCAM_WaitSeconds(6, true);
#ifdef DEBUG
  fprintf(PAYLOAD,"Turning on LUP 5v\n\r");
#endif
  EPS_write(6,0);  //  --> turn on LUP 5v
  D_XCAM_WaitSeconds(6, true);
  EPS_check(1,1);
  return;
}

/**
* @brief: This will turn the Power off on the XCAM than turn it back on
* @param: none
* @retvalue: none
*/ 
void D_XCAM_Power_Cycle(void){
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  D_XCAM_Power_Off();
  D_XCAM_WaitSeconds(5, true);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  D_XCAM_Power_On();
  D_XCAM_WaitSeconds(30, 1);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
}


/** 
*  @brief: This function Initializes the XCAM by Sending the Payload initialize command. Leave it alone for 5 sec
*  @param: none
*  @retvalue: (1) if the first Init did not run sucessfully
*             (2) if the second Init did not run sucessfully
*             (0) if the Init ran sucessfully
* @note: This function is Not used by the Main Function.
*/
uint8_t D_XCAM_Init(void){
//  The start-up sequence for C3D is as follows:
//1) Power on the 3V3 line to initialise the I2C buffer
//2) Power on the 5V line to boot the FPGA
//3) Send the payload a ‘payload initialise command’ to complete the start-up sequence. This will boot the remaining peripheral devices.
  if (D_XCAM_SendInitCommand())
    return 1;
//4) Leave for five seconds to allow the secondary CPU to boot
  D_XCAM_WaitSeconds(5, true);
// Need to send a new init command.
  if (D_XCAM_SendInitCommand())
    return 2;
  return 0;
}


/**
 * @brief : This function Will Analyze the Status of the XCAM and return the number of packets waiting in the payload memory
*  @param *status: a pointer to where the status lives
*  @param prioriryData: pointer to the register to store the number of priority data packets
*  @retvalue: opperationFlag Hex info about the current opperation Flags
*/
uint16_t D_XCAM_AnalyzeStatus(uint8_t *status, uint16_t *priorityData, bool *Error_Flag)
{
  uint8_t OperationMode = status[2];
  uint16_t OperationFlag = (status[3] << 8) | status[4];
  uint16_t PriorityData = (status[5] << 8) | status[6];
  uint32_t TotalData = (status[7] << 24) | (status[8] << 16) | (status[9] << 8) | status[10];
  *Error_Flag = false;
  char stringBuffer[150];
  if (OperationMode){
    sprintf(stringBuffer, "\tImaging Mode, Flags: ");
    
  }else{
    sprintf(stringBuffer, "\t\tStandby Mode, Flags: ");
  }
  
  if (OperationFlag & 0x01){
    strcat(stringBuffer, " BUSY\n\r");
  }
  if (OperationFlag & 0x02){
    strcat(stringBuffer, " FINISHED\n\r");
  }
  if (OperationFlag & 0x08){
    strcat(stringBuffer, " NOPACKETS\n\r");
  }
  if (OperationFlag & 0x10){
    //Last opperation Failed
    strcat(stringBuffer, " OPERR\n\r");
    uint8_t status[22] = {0};
    D_XCAM_ReadErrorParameter(status);
    D_XCAM_AnalyzeError(status);
    *Error_Flag = true;
    
  }
  if (OperationFlag & 0x20){
    //Invalid Opperation Mode
    strcat(stringBuffer, " INVALID\n\r");
    *Error_Flag = true;
  }
  //Print Data To HK
  //print(stringBuffer);

  if (*Error_Flag == true){
    print("An Error Happened When Analyzing the Status Code\n\r");
    print(stringBuffer);
  }
  *priorityData = PriorityData;
  //sprintf(stringBuffer, "\t\tPriority Data: %d\n\r", PriorityData);
  //print(stringBuffer);
  //sprintf(stringBuffer, "\t\tTotal Data: %d\n\r", TotalData);
  //print(stringBuffer);
  return OperationFlag;
}


/**
* @brief: This Finction Will get the operation Status of the XCAM and Return it to the status address
* @param status: pointer to where to save the Data
* @retval: returns based on Sucess (0) or failure (1) did not transmit (2) did not recieve 
*/
uint8_t D_XCAM_GetStatus(uint8_t *status)
{
  uint8_t txbuf[4] = {0};
  uint8_t i;

  for (i=0;i<22;i++)
    status[i] = 0;
 
  txbuf[0] = 0x91;
  txbuf[1] = 1;

  D_XCAM_SetCRC(txbuf, 4);
  if (D_XCAM_DEBUG)
    //fprintf(PAYLOAD, "Requesting camera status.\n\r");
  if (D_XCAM_transmit(txbuf, 4))
    return 1;
  osDelay(3);
  if (D_XCAM_receive(status, 22, true))
    return 2;

  if (status[4] == 0x92)
        print("what");

  if (D_XCAM_RAWSTATUS)
  {
    fprintf(PAYLOAD, "Status: 0x");
    for (i=0; i<22; i++)
      fprintf(PAYLOAD, "%02x ", status[i]);
    fprintf(PAYLOAD, "\r\n");
  }
  return 0;
}    



/**
 * @brief: This will read The Error Register
 * @param: none
 * @retval: returns 0 if it worked well 1 if did not transmit 2 if did not revieve
 */
uint8_t D_XCAM_ReadErrorParameter(uint8_t *status){
  uint8_t i;
  uint8_t txbuf[5] = {0};
  char stringBuffer[150] = {0};
  //Request Error Status
  txbuf[0] = 0x94;
  txbuf[1] = 1;
  txbuf[2] = 0x15;
  D_XCAM_SetCRC(txbuf, 5);

  for (i=0;i<22;i++)
    status[i] = 0;
//  if(DEBUG)
//    print("Requesting camera error.\n\r");
  if(D_XCAM_transmit(txbuf, 5)){
    print("Could not transmit\r\n");
    return 1;
  }

  osDelay(4);
  if (D_XCAM_receive(status, 22, true)){
    print("Could not receive\r\n");
    return 2;
  }

  if(D_XCAM_RAWSTATUS)
  {
    char tempBuffer[5];
    strcat(stringBuffer, "Status: 0x");
    for (i=0; i<22; i++){
      sprintf(tempBuffer, "%02x ", status[i]);
      strcat(stringBuffer, tempBuffer);
    }
    strcat(stringBuffer, "\r\n");
    print(stringBuffer);
    
  }

  return 0;


}



/**
* @brief:Analyze the error status of the XCAM *
* @param: status: pointer to where the error status lives
* @retval: OperationFlag: The Curent Opperation Flag
* @note Dont have time to figure out how to handle all the error Statuses properly will just reset the camera for now
*/
uint16_t D_XCAM_AnalyzeError(uint8_t *status){
  //Allocate Memory for the error status
  char stringBuffer[200] = {0};
  strcat(stringBuffer, "Analyzing Error Status\n\r");
  uint16_t OperationFlag = (status[2] << 8) | status[3];
  
  if (OperationFlag & 0x01){
    strcat(stringBuffer, " Insufficient Memory\n\r"); 
  }
  if (OperationFlag & 0x02){
    strcat(stringBuffer, " Auto Exposure Failure\n\r");
  }
  if (OperationFlag & 0x03){
    strcat(stringBuffer, " Grab Timeout\n\r");
  }
  if (OperationFlag & 0x05){
    strcat(stringBuffer, " CIS Setup Failure\n\r");
  }
  if (OperationFlag & 0x06){
    strcat(stringBuffer, " CIS Grab Failure\n\r");
  }
    if (OperationFlag & 0x07){
    strcat(stringBuffer, " Invalid Parameter Combination\n\r"); 
  }
  print(stringBuffer);
  
  return OperationFlag;
}

/**
 * @brief : This will pull a single image packet from the XCAM
 * @param buffer : Pointer to the buffer
 * @return (1) if it did not transmit (2) issue with HAL (3) if it did not receive (0)everything Went Well
 */ 
uint8_t D_XCAM_GetImageSPI(uint8_t *buffer)
{
  HAL_StatusTypeDef SPI_ret;
  uint8_t txbuf[5] = {0};
  //Setup the Command
  txbuf[0] = 0x97;
  txbuf[1] = 1;
  txbuf[2] = 0x00;

  D_XCAM_SetCRC(txbuf, 5);
  if (D_XCAM_DEBUG)
    fprintf(PAYLOAD, "D");
  if (D_XCAM_transmit(txbuf, 5))
    return 1;
  osDelay(4);
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_ret = HAL_SPI_Receive(&hspi1, buffer, 260, 3000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  
  if ((SPI_ret == HAL_OK) && (D_XCAM_HALOK))
    fprintf(PAYLOAD,"\tSPI_Receive return was HAL_OK\n\r");
  if (SPI_ret != HAL_OK)
  {
    fprintf(PAYLOAD,"\tSPI_Receive return was NOT HAL_OK\n\r");
//    HAL_Recovery_Tree(SPI_ret, 0);
    return 2;
  }

  if (D_XCAM_receive(txbuf, 0, true))  // it doesn't matter which buffer
    return 3;

  if (D_XCAM_ValidateCRC(buffer, 260) == false)
  {
    fprintf(PAYLOAD, "CRC FAIL %x\r\n",buffer[0]);
  }

  D_XCAM_PrintACKOrResponse(buffer, 260);
  return 0;
}


/**
 * @brief : This will pull a single image packet from the XCAM
 * @param buffer : Pointer to the buffer
 * @return (1) if it did not transmit (2) issue with HAL (3) if it did not receive (0)everything Went Well
 */ 
uint8_t D_XCAM_GetImageI2C(uint8_t *buffer){

  uint8_t txbuf[4] = {0};

  txbuf[0] = 0x95;
  txbuf[1] = 1;

  D_XCAM_SetCRC(txbuf, 4);
  if (D_XCAM_DEBUG)
    fprintf(PAYLOAD, "Sending I2C Download Image Command\n\r");
  if (D_XCAM_transmit(txbuf, 4))
    return 1;
  osDelay(3);
  if (D_XCAM_receive(buffer, 260, true))
    return 2;

  D_XCAM_PrintACKOrResponse(buffer, 260);
  
  return 0;
}


/**
 * @brief : This will Send an initializiation command to XCAM
 * @param none
 * @return (1) if it did not transmit (2) issue with HAL (0)everything Went Well
 */ 
uint8_t D_XCAM_SendInitCommand(void)
{
  return D_XCAM_SendInitOrUpdate(true, false);
}

/**
 * @brief : This will Send an Enable Imaging command to XCAM
 * @param none
 * @return (1) if it did not transmit (2) issue with HAL (0)everything Went Well
 */ 
uint8_t D_XCAM_EnableImagingMode(void)
{
  D_XCAM_SetParameter(XCAM_SENSOR,XCAM_sensor);
  D_XCAM_SetParameter(XCAM_TIMEOUT,XCAM_timeout);
  D_XCAM_SetParameter(XCAM_TIMEOUT, XCAM_timeout);
  D_XCAM_SetParameter(XCAM_WINDOW, XCAM_window);
  D_XCAM_SetParameter(XCAM_INTTIMEFRAC, XCAM_inttimefrac);
  D_XCAM_SetParameter(XCAM_COMPRESS, XCAM_compress);
  D_XCAM_SetParameter(XCAM_THUMB, XCAM_thumb);
  return D_XCAM_SendInitOrUpdate(false, true);
}

/**
 * @brief : Send an initializiation or Update Command to XCAM
 * @param init : true for init, false for update
 * @param imagingmode : true for imaging, false for non-imaging
 * @return (1) if it did not transmit (2) issue with HAL (0)everything Went Well
 */ 
uint8_t D_XCAM_SendInitOrUpdate(bool init, bool imagingmode)
{
  uint8_t txbuf[25] = {0};
  uint8_t rxbuf[5] = {0};
  unsigned long ukube_time;

  if (init)
  {
    if (D_XCAM_DEBUG)
      fprintf(PAYLOAD, "Sending Init Command\n\r");
    txbuf[0] = 0x90;
  }
  else
  {
    if (D_XCAM_DEBUG)
      fprintf(PAYLOAD, "Sending Update Command\r\n");
    txbuf[0] = 0x92;
  }
  txbuf[1] = 1;
  if (imagingmode)
  {
    if (D_XCAM_DEBUG)
      fprintf(PAYLOAD, " and setting imaging mode.\n\r");
    txbuf[2] = 1;
  }
  else
  {
    if (D_XCAM_DEBUG)
      fprintf(PAYLOAD, " and setting standby mode.\n\r");
    txbuf[2] = 0;
  }
  
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  HAL_RTC_GetTime(&hrtc,&sTime,calendar_format); // must be before GetDate
  HAL_RTC_GetDate(&hrtc,&sDate,calendar_format);
  fprintf(PAYLOAD,"Setting XCAM time to %02d-%02d %02d:%02d:%02d\n\r",
          sDate.Month,sDate.Date,sTime.Hours,sTime.Minutes,sTime.Seconds);

  ukube_time =   (long)sTime.Seconds
               +((long)sTime.Minutes*60L)
               +((long)sTime.Hours*3600L)
               +((long)sDate.Date*24L*3600L)
               +((long)sDate.Month*30L*24L*3600L);
               
  txbuf[5] = (uint8_t)((ukube_time >> 24) & 0xff);
  txbuf[6] = (uint8_t)((ukube_time >> 16) & 0xff);
  txbuf[7] = (uint8_t)((ukube_time >>  8) & 0xff);
  txbuf[8] = (uint8_t)((ukube_time      ) & 0xff);

  D_XCAM_SetCRC(txbuf, 25);
  if (D_XCAM_transmit(txbuf, 25))
    return 1;
  osDelay(3);
  if (D_XCAM_receive(rxbuf, 5, false))
    return 2;

  D_XCAM_PrintACKOrResponse(rxbuf, 5);
  return 0;
}

/****************************************************************
 * @brief : This will Send a command to XCAM to set a given Parameter
 * @param ID : Parameter to set
 * @param value : Value to set it to
 * @return (1) if it did not transmit (2) issue it ded not recieve (0)everything Went Well
 */
uint8_t D_XCAM_SetParameter(uint8_t ID, uint16_t value)
{
  uint8_t txbuf[7] = {0};
  uint8_t rxbuf[5] = {0};

  txbuf[0] = 0x93;
  txbuf[1] = 1;
  txbuf[2] = ID;
  txbuf[3] = (uint8_t) ((value >> 8) & 0xff);
  txbuf[4] = (uint8_t) (value & 0xff);

  D_XCAM_SetCRC(txbuf, 7);
  if (D_XCAM_DEBUG)
    fprintf(PAYLOAD, "Setting parameter %02x to %d.\n\r", ID, value);

  if (D_XCAM_transmit(txbuf, 7)){
      fprintf(PAYLOAD, "Could not Transmit to XCAM\n\r");
      return 1;
  }
    osDelay(3);
  if (D_XCAM_receive(rxbuf, 5, false)){
      fprintf(PAYLOAD, "Could not recieve from XCAM\n\r");
      return 2;
  }
  D_XCAM_PrintACKOrResponse(rxbuf, 5);
  return 0;
}

/****************************************************************
 * @brief : This will Send a command to XCAM to set a given Parameter
 * @param ID : Parameter to set
 * @return (1) if it did not transmit (2) issue it ded not recieve (0)everything Went Well
 */
/*
char * D_XCAM_GetParameter(uint8_t ID){
  uint8_t i;
  uint8_t txbuf[5] = {0};
  uint8_t status[22] = {0};

  txbuf[0] = 0x94;
  txbuf[1] = 1;
  txbuf[2] = ID;
  D_XCAM_SetCRC(txbuf, 5);

  for (i=0;i<22;i++)
    status[i] = 0;
  #ifdef DEBUG
    fprintf(PAYLOAD, "Requesting camera error.\n\r");
  #endif
  
  if (D_XCAM_transmit(txbuf, 5)){
      #ifdef DEBUG
        fprintf(PAYLOAD,"Could not transmit\r\n");
      #endif
  }

  osDelay(3);

  char Parameter[50] = {0};
  char tempBuffer[20];
  if (D_XCAM_receive(status, 22, true)){
      sprintf(Parameter,"Could not receive\r\n");
  }


  if (D_XCAM_RAWSTATUS){
    sprintf(Parameter, "Status: 0x");
    for (i=0; i<22; i++)
    {
      sprintf(tempBuffer, "%02x ", status[i]);
      strcat(Parameter,tempBuffer);
    }
    strcat(Parameter, "\r\n");
  }

  return Parameter;
}
*/

void D_XCAM_SetCRC(uint8_t* data, size_t len)
// length of total packet including CRC
{
  uint16_t crc_raw;
  crc_raw = D_XCAM_crc16(65535, data, len-2);
  data[len-2] = (uint8_t)((crc_raw >> 8) & 0xff);
  data[len-1] = (uint8_t)( crc_raw       & 0xff);
}

bool D_XCAM_ValidateCRC(uint8_t* data, size_t len)
// length of total packet including CRC
{
  uint16_t crc_raw;
  uint8_t crc1, crc2;
  crc_raw = D_XCAM_crc16(65535, data, (len-2));
  crc1 = (uint8_t)((crc_raw >> 8) & 0xff);
  crc2 = (uint8_t)( crc_raw       & 0xff);
  if ((data[len-2] == crc1) && 
      (data[len-1] == crc2))
    return true;
  return false;
}


uint16_t D_XCAM_crc16(uint16_t seed, uint8_t *pBuffer, int length)
{
  uint16_t crc_lut[256] = {
      0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
      0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
      0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
      0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
      0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
      0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
      0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
      0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
      0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
      0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
      0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
      0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
      0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
      0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
      0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
      0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
      0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
      0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
      0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
      0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
      0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
      0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
      0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
      0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
      0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
      0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
      0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
      0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
      0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
      0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
      0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
      0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

  uint16_t crc = 65535;
  uint16_t data, temp, temp_rs, temp_xor, temp_ls, p;

  for (p=0; p<length; p++)
  {
    data     = pBuffer[p];
    temp_rs  = crc >> 8;
    temp_xor = temp_rs ^ data;
    temp     = temp_xor & 0xff;
    temp_ls  = crc << 8;
    crc      = crc_lut[temp] ^ temp_ls;
  }
  return crc;
}


uint8_t D_XCAM_transmit(uint8_t *buffer, size_t len)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  ret = HAL_I2C_Master_Transmit(&hi2c3, D_XCAM_ADDRESS << 1,
                                buffer, len, 100);
  if ((ret == HAL_OK) && D_XCAM_HALOK)
    fprintf(PAYLOAD,"\tXCAM_transmit return was HAL_OK\n\r");
  if (ret != HAL_OK)
  {
    fprintf(PAYLOAD,"\tXCAM_transmit return was NOT HAL_OK\n\r");
//    fprintf(PAYLOAD, "\tAttempting to recover HAL");
//    HAL_Recovery_Tree(ret);
    //Jump to HAL recovery
    return ret;
  }
  return 0;
}


uint8_t D_XCAM_receive(uint8_t *buffer, size_t len, bool ack)
{
//  fprintf(PAYLOAD, "\tXCAM_receive\n\r"); // this probably slows things down
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint8_t buf[5] = {0};
  if (len > 0)
  {
    ret = HAL_I2C_Master_Receive(&hi2c3, D_XCAM_ADDRESS << 1,
                                 buffer, len, 100);
    if ((ret == HAL_OK) && D_XCAM_HALOK)
      fprintf(PAYLOAD,"\tXCAM_receive return was HAL_OK\n\r");
    if (ret != HAL_OK)
    {
      fprintf(PAYLOAD,"\tXCAM_receive return was NOT HAL_OK\n\r");
//      HAL_Recovery_Tree(ret);
      return ret;
    }
    
    // there's a bug in the camera firmware where a command 0x95 returns a 0x91 as the first byte    
    if ((len == 260) && (buffer[0] = 0x91))
        buffer[0] = 0x95;
    if (D_XCAM_ValidateCRC(buffer, len) == false)
    {
      print("WARNING: response failed CRC check\n\r");
      //fprintf(PAYLOAD,"%x\n",buffer[0]);
    }
  }
  //For Type 2 PackagesWe need to Send an Ack
  if (ack) // if we need to send an acknowledgement packet
  {
    buf[0] = buffer[0];
    buf[1] = 1;
    buf[2] = 0x7E;

    D_XCAM_SetCRC(buf, 5);
    ret = HAL_I2C_Master_Transmit(&hi2c3, D_XCAM_ADDRESS << 1,
                                  buf, 5, 100);
    if ((ret == HAL_OK) && D_XCAM_HALOK)
      fprintf(PAYLOAD,"\tACK return was HAL_OK\n\r");
    if (ret != HAL_OK)
    {
      fprintf(PAYLOAD,"\tACK return was NOT HAL_OK\n\r");
//      HAL_Recovery_Tree(ret);
      return 2;
    }
  }
  return 0;
}


void D_XCAM_WaitSeconds(uint16_t numSeconds, bool verbose)
{
  uint16_t ii;
  if (verbose)
    fprintf(PAYLOAD,"Waiting %d seconds...",numSeconds);
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  for (ii=0; ii<(10*numSeconds); ii++)
  {
    osDelay(100);
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    if ((verbose) && ((ii%10) == 0))
      fprintf(PAYLOAD,"-%d ",ii/10);
  }
  if (verbose)
    fprintf(PAYLOAD,"\n\r");

}


void D_XCAM_PrintACKOrResponse(uint8_t *buffer, size_t len)
{
  if ((len == 5) && (buffer[2] == 0x7e))
  {
    fprintf(PAYLOAD, "Response: ACK\n\r");
  }
  else
  {
    #ifdef DEBUG
//      uint16_t i;
      //fprintf(PAYLOAD, "Response: 0x");
      //for (i=0; i<len; i++)
          //fprintf(PAYLOAD, "%02x ", buffer[i]);
      //fprintf(PAYLOAD, "\n\r");
    #endif
  }
}
