#include "SD_handler.h"
#include "TaskMonitor.h"
#include "EPS.h"
#include "time.h"
#include "defs.h"





#include <time.h>

/**
 * @brief This Function Makes the House Keeping File
 * @param: *FileName: The Path Of the File.
 * @retval: (3) if it fails to mount the SD Card
 *          (2) if it fails to open the file, 
 *          (1) if it fails to write to the file, 
 *          (0) if it successfully makes the file and writes to it.
 */
uint8_t SD_Make_File(const char *FileName){
    FRESULT res; /* FatFs function common result code */
    uint32_t byteswritten, bytesread; /* File write/read counts */
    uint8_t val;

    //Attempt to Mount File
    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
      //if it didnt work
      fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
      val = 3;
      goto error;
    }

    //Open file for writing (Create)

    res = f_open(&SDFile, FileName, FA_OPEN_ALWAYS | FA_WRITE);
    if (res == FR_NO_PATH){
        BuildPath(FileName);
        res = f_open(&SDFile, FileName, FA_CREATE_ALWAYS | FA_WRITE);
    }
    //Still Couldnt Open the File
    if(res != FR_OK){
        fprintf(PAYLOAD, FileName);
        fprintf(PAYLOAD,"Couldnt Open File for writing\r\n");
        fprintf(PAYLOAD, "%02x ", res);
        fprintf(PAYLOAD, "\r\n");
        fprintf(PAYLOAD,"Couldnt Make Dir\r\n");
        val = 2;
        goto error;
    }

    //Close the File 
    f_close(&SDFile);
    fprintf(PAYLOAD, "Successfully Written To SD Card!\r\n");
    val = 0;
    
    error:
    //Unmount File
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
    return val;
}


/**
 * @brief This Function Makes the House Keeping File
 * @param: *FileName: The Path Of the File.
 * @retval: (3) if it fails to mount the SD Card
 *          (2) if it fails to open the file, 
 *          (1) if it fails to write to the file, 
 *          (0) if it successfully makes the file and writes to it.
 */
uint8_t SD_Append_String_File(const char *FileName, char *data, uint16_t len){
    FRESULT res; /* FatFs function common result code */
    uint32_t byteswritten, bytesread; /* File write/read counts */
    uint8_t val;    
    //Attempt to Mount File
    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
      
      //if it didnt work
      fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
      Error_Handler();
      val = 3;
      goto error;
    
    }  
    
    //Open file for writing (Create)
    
    if(f_open(&SDFile, FileName, FA_OPEN_APPEND | FA_WRITE) != FR_OK){
        Error_Handler();
        fprintf(PAYLOAD,"Couldnt Open File for writing\r\n");
        val = 2;
        goto error;
    }
    
    //Write to the text file
    res = f_write(&SDFile, data, len, (void *)&byteswritten);
    if((byteswritten == 0) || (res != FR_OK)){
        Error_Handler();
        #ifdef DEBUG
            fprintf(PAYLOAD,"Couldnt Write to File\r\n");
        #endif
        f_close(&SDFile);
        val = 1;
        goto error;
    }
    
    //Close the File
    f_close(&SDFile);
    #ifdef DEBUG
        fprintf(PAYLOAD, "Sucessfully Written To SD Card!\r\n");
    #endif
    val = 0;
    
    error:
    //Unmount File
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
    return val;
}


/**
 * @brief This Function Makes the House Keeping File
 * @param: *FileName: The Path Of the File.
 * @retval: (3) if it fails to mount the SD Card
 *          (2) if it fails to open the file, 
 *          (1) if it fails to write to the file, 
 *          (0) if it successfully makes the file and writes to it.
 */
uint8_t SD_Append_Data_File(const char *FileName, uint8_t *data, uint16_t len){
    FRESULT res; /* FatFs function common result code */
    uint32_t byteswritten, bytesread; /* File write/read counts */
    uint8_t val;    
    //Attempt to Mount File
    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
      
      //if it didnt work
      fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
      Error_Handler();
      val = 3;
      goto error;
    
    }  
    
    //Open file for writing (Create)
    if(f_open(&SDFile, FileName, FA_OPEN_APPEND | FA_WRITE) != FR_OK){
        Error_Handler();
        fprintf(PAYLOAD,"Couldnt Open File for writing\r\n");
        val = 2;
        goto error;
    }
    
    //Write to the text file
    res = f_write(&SDFile, data, len, (void *)&byteswritten);
    if((byteswritten != len) || (res != FR_OK)){

        #ifdef DEBUG
            fprintf(PAYLOAD,"Couldnt Write to File\r\n");
        #endif
        f_close(&SDFile);
        val = 1;
        goto error;
    }
    
    //Close the File
    f_close(&SDFile);
    #ifdef DEBUG
        fprintf(PAYLOAD, "Successfully Written To SD Card!\r\n");
    #endif
    val = 0;
    
    error:
    //Unmount File
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
    return val;
}


/**
 * @brief This Function Makes A Directory on the SD CARD
 * @param: *DirName: The Path Of the File.
 * @retval: (2) if it fails to mount the SD Card 
 *          (1) if it fails to make Dir, 
 *          (0) if it successfully makes the file and writes to it.
 */
uint8_t SD_Make_Dir(const char *DirName){
    FRESULT res; /* FatFs function common result code */
    uint8_t val;
    //Attempt to Mount File
    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
      //if it didnt work
      fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
      Error_Handler();
      val = 2;
      goto error;
    }
    fprintf(PAYLOAD, "\n\r");
    fprintf(PAYLOAD, DirName);
    fprintf(PAYLOAD, "\n\r");
    //Make Dir

    BuildPath(DirName);
    // res = f_mkdir(Dirname);

    // if(res != FR_OK){
    //     fprintf(PAYLOAD, "%02x ", res);
    //     fprintf(PAYLOAD, "\r\n");
    //     fprintf(PAYLOAD,"Couldnt Make Dir\r\n");
    //     val = 1;
    //     goto error;
    // }
    
    //Close the File
    fprintf(PAYLOAD, "Sucessfully Made Dir!\r\n");
    val = 0;
    
    error:
    //Unmount File
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
    return val;
}


/**
  * @brief: This Function Creates The File System
  * @param: None
  * @retval: (0) if successfully created the file system
  *          (1) if failed to mount SD
  *          (2) if failed to create the file system
*/
uint8_t SD_FileSystem_Create(void){
  uint8_t status = 0;
  uint8_t rtext[_MAX_SS];/* File read buffer */
  //struct tm = Get_Date_Time();
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
    //if it didnt work
    fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
    return 1;
  }
  if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK){
    fprintf(PAYLOAD, "Couldn't create file system\r\n");
    //Unmount File
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
    return 2; 
  }

  fprintf(PAYLOAD, "File System Created\r\n");
  //Unmount File
  f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
  return 0;
}


/**
 * @brief Allocate buffer for the file name
 * @param str String To allocate Memory for 
 */
char *strmalloc(char *str)
{
  char *s = malloc(strlen(str) + 1);
  if (s)
    strcpy(s, str);
  return(s);
}
 


/*************************************************
 * @brief Recursivley Build a path Tree to the SD Card
 * @param path Entire Path like /My/Path/To/File.txt 
 ***************************************************/
void BuildPath(char *path){
  TaskMonitor_IamAlive(TASK_MONITOR_DEFAULT);
  FRESULT res;
  char *s = strmalloc(path);
  int i = strlen(s);

// Find decomposition point
    while(i && s[i-1] != '/') 
        i--;
 
    if (i) // Move to '/'
        i--;
 
    if (i){
        s[i] = 0; // replace '/' with NUL
        res = f_mkdir(s);
        if (res != FR_OK){
            #ifdef DEBUG
                fprintf(PAYLOAD, "%02x ", res);
                fprintf(PAYLOAD, "\r\n");
                fprintf(PAYLOAD,"Couldnt Make Dir\r\n");
                fprintf(PAYLOAD, "Couldn't make dir %s\r\n", s);
            #endif
    }

    if (res == FR_NO_PATH){
      BuildPath(s); // Drop down a level, and build that
      res = f_mkdir(s); // Try again

    }
 
    if (res == FR_OK){
        #ifdef DEBUG
            fprintf(PAYLOAD, "Created path '%s'\n\r", s);
        #endif
    }
  }
    //Free Up the Mem
    free(s);
}


/****************************************************************
 * @brief: Check if a Filename Exists on the SD
 * @param: FileName: Name of the file to check
 * @retval: (1) if file exists
 *          (0) if file does not exist
 *         (2) if failed to mount SD
 ****************************************************************/
uint8_t SD_File_Exists(const char *FileName){
  uint8_t status = 0;
  FILINFO fno;
  FRESULT res;
  //Mount The SD
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
    //if it didnt work
    fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
    return 2;
  }
  res = f_stat(FileName, &fno);
  if(res == FR_OK){
    status = 1;
  }
  //Unmount SD
  f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
  return status;
}

/****************************************************************
 * @brief: Get a list of Files on the SD
 * @param: FILINFO* fno - File information structure to store the file information
 * @retval: (1) no files
 *          (0) if things went Sucessfully
 *          (2) if failed to mount SD
 ****************************************************************/
uint8_t SD_GetFiles(FILINFO *fnoList){
  uint8_t status = 0;
  FILINFO fno;
  FRESULT res;
  DIR dir;

  //Mount The SD
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
    //if it didnt work
    fprintf(PAYLOAD, "Couldn't mount SD Card\r\n");
    return 2;
  }
  //TODO -- Software Freezing here unsure Why
  res=f_opendir(&dir,"/");
  uint16_t i = 0;
  do{
     res = f_readdir(&dir, &fno);
     if((res==FR_OK)&&(fno.fname[0]!= 0)){
        fprintf(PAYLOAD, "File found: %s\n", fno.fname); // Print File Name
        fnoList[i] = fno;
        i++;
     }
  }while(fno.fname[0]!= 0);
//  f_opendir(&SDPath, 0);   // Open Root
//  uint8_t i = 0;
//  do{
//      f_readdir(&SDPath, &fno);
//      if(fno.fname[0] != 0)
//
//          fprintf(PAYLOAD, "File found: %s\n", fno.fname); // Print File Name
//          fnoList[i] = fno;
//          i++;
//  } while(fno.fname[0] != 0);

//  res = f_opendir(&SDFatFS, fno);
//    if(res == FR_OK){
//      status = 1;
//    }
    //Unmount SD
    f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
 return status;
}

