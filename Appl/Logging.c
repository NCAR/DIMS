#include "Logging.h"
#include <time.h>




//Define Global
const char HouseKeepingName[10];
bool HouseKeeping_File_Created = false;

/**
 * @brief : This weill make the File System For the SD Card
 * @retval: (1) if it fails to Make the File System
 *          (2) if it fails to Make the Unique Run ID Dir
 *          (3) if it fails to Make the HouseKeeping File
 *          (0) if it successfully makes the File System
 */
uint8_t Setup_SD(void){
    //Create a fatfs File System

    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    if(SD_FileSystem_Create()){
        print("Couldn't create file system\r\n");
        return 1;
    }

    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);

//TODO Need to Get this working
    //find a uniq id for the HK data
    if(get_next_housekeeping_file_id(&HouseKeepingName[0])){
        print("Couldn't get a unique ID for the HK file\r\n");
        return 2;
    }
    
    //Make the Housekeeing File
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    if(Make_HouseKeeping(HouseKeepingName)){
        //if DEBUG is defined print message to terminal
        print("Couldn't make HK file\r\n");
        
        return 3;
    }
    
    
   
    print("Made Housekeeping File\r\n");
    print("Successfully Created File System\r\n");
    HouseKeeping_File_Created = true;
    return 0;
}

/**
 * @brief : This will make the House Keeping File For the SD Card Issue With this Function
 *
 * @param : *FileName: The Path Of the File.
 * @retval: (1) if it fails to Make the File 
 *          (0) if it successfully makes the File
 */
uint8_t Make_HouseKeeping(const char *FileName){
    
    if(SD_Make_File(FileName)){
        print("Couldn't make HK file\r\n");
        //Error handler
        return 1;
    }
    print("Made Housekeeping File\r\n");
    HouseKeeping_File_Created = true;
    return 0;
}


/****************************************************************
 * @brief : This Print to the Comport or will write to the Houskeeping File if its setup
 * @param : *String: Const Char string to print and Write.
 * @retval: (1) if it fail 
 *          (0) if its successful
 ****************************************************************/
uint8_t print(const char *String){
    //If the HK file is setup write to it
    if(HouseKeeping_File_Created){
        Write_To_HK(String);
    }

    //Print to the COM Port
    #ifdef DEBUG
        fprintf(PAYLOAD, String);
    #endif
    return 0;
}

/****************************************************************
 * @brief : This will write to the House Keeping File For the SD Card
 * @param : *String: The log message to place in the file.
 * @retval: (1) if it fails to write to the File
 *         (0) if it successfully writes to the File
 * ***************************************************************/
void Write_To_HK(const char *String){
    //Prefix date Time to String
    const char DateTime[100];
    get_dateString(&DateTime[0]);
    strcat(DateTime, ": ");
    strcat(DateTime, String);
    strcat(DateTime, "\r\n");
    //Write to File
    if(SD_Append_String_File(HouseKeepingName, &DateTime[0], strlen(DateTime))){
        #ifdef DEBUG
            fprintf(PAYLOAD, "Couldn't write to HK file\r\n");
        #endif
    }
    
    #ifdef DEBUG
        fprintf(PAYLOAD, "Successfully wrote to HK file\r\n");
    #endif
}



/****************************************************************
 * @brief : This will get the Next Run Id available in the System. by searching through existing files with the label xxxxxx.hk where x is a number. Using SD_GetFiles to get the list of files in the system.
 * @param : *FileName: Where to Store the Next available File name in the System.
 * @retval: (1) if it fails to find the next available file name
 *         (0) if it successfully finds the next available file name
 *         TODO -- ISSues Here. With pulling files in Dir
 * ***************************************************************/
 uint8_t get_next_housekeeping_file_id(const char *String){
    uint8_t filename_iter = 0;
    const char filename[10];
    bool found_filename = false;

    while(!found_filename){
        sprintf(filename, "%06d.hk", filename_iter);
        if(SD_File_Exists(filename)==0){
            found_filename = true;
        }
    }
    //Get the list of files in the system
    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    const char buffer[50];
    sprintf(buffer, "Found Free File Name: %s\r\n", filename);
    print(buffer);
    sprintf(String, "%s", filename);
    return 0;
    
 }

/****************************************************************
 * @brief : This will get the Next Image ID available in the System. by searching through existing files with the label xxxxx.raw where x is a number. Using SD_GetFiles to get the list of files in the system.
 * @param : *ImageFileName: Where to Store the Next available File name in the System.
 * @param : *HeaderFileName: Where to Store the Next available Header File name in the System.
 * @retval: (1) if it fails to find the next available file name
 *         (0) if it successfully finds the next available file name
 * ***************************************************************/
uint8_t get_next_image_id(const char *ImageFileName, const char *HeaderFileName){
    uint8_t filename_iter = 0;
    const char image_filename[10];
    const char header_filename[10];    
    bool found_filename = false;

    while(!found_filename){
        sprintf(image_filename, "%05d.raw", filename_iter);
        if(SD_File_Exists(&image_filename[0]==0)){
            found_filename = true;
        }else{
            filename_iter++;
        }
    }
    //Get the list of files in the system

    TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
    const char buffer[50];
    sprintf(header_filename, "%05d.txt", filename_iter);
    sprintf(buffer, "Found Free File Name: %s\r\n", image_filename);
    print(buffer);
    sprintf(ImageFileName, "%s", image_filename);
    sprintf(HeaderFileName, "%s", header_filename);
    return 0;
    
    
 }




/**
 * @brief Generate a 5 digit random number in string Format
 * @param NumString: The string to store the number
 * 
 */
void get_random_string(char* NumString){
    /* Global Vars */
    uint32_t random_number;
    /* Get a random number */
    srand(time(NULL));
    random_number = rand()%100000;
    /* Convert the number to a string */
    sprintf(NumString, "%d", random_number);
    return;
}


/**
 * @brief Get the dateString object
 * @param dateString location to Store the Date Time 
 */
void get_dateString(char* dateString){
    /* Global Vars */

/* Global Vars */
    RTC_TimeTypeDef currentTime;
    RTC_DateTypeDef currentDate;
    time_t timestamp;
    struct tm currTime;
    /* Get the current time */
    HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
    /* Convert the current time to a timestamp */
    currTime.tm_year = currentDate.Year + 100;
    currTime.tm_mon = currentDate.Month - 1;
    currTime.tm_mday = currentDate.Date;
    currTime.tm_hour = currentTime.Hours;
    currTime.tm_min = currentTime.Minutes;
    currTime.tm_sec = currentTime.Seconds;
    currTime.tm_isdst = -1;
    timestamp = mktime(&currTime);
    /* Convert the timestamp to a string */
    fprintf(PAYLOAD, "%Y-%m-%d %H:%M:%S", localtime(&timestamp));
    fprintf(PAYLOAD, "Size of DateString %i", sizeof(dateString));
    strftime(dateString, sizeof(dateString), "%Y-%m-%d %H:%M:%S\0", localtime(&timestamp));
}
