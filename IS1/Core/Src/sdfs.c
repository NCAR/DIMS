/*
 * sdfs.c
 * 
 *  A library to handle all of the SD card Functions
 *  Created on: Jan 11, 2021
 *      Author: damonb
 *      edited by mjeffers
 */

#include "sdfs.h"
#include <string.h>
#include "gps.h"
#include "defs.h"

FATFS FS;

/**************
 * @brief Find a Free Directory File name
 * @param state: pointer to the state struct of the System
 * @retval None
*/
void SDFS_FindNewDirectory(struct sState *state)
{
  uint16_t num;
  for (num=0; num<10000; num++)
  {
    snprintf(state->RootPath, 50, "%05i", num);
    if (!(SDFS_MakeDir(state->RootPath)))
      break;
  }
}

/**************
 * @brief Make a Directory File
 * @param path: pointer to the path of the directory
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_MakeDir(char path[])
{
  FRESULT fres;

  printf("Attempting to create path %s", path);
  fres = f_mkdir(path);

  if (fres == FR_OK)
  {
    printf(" - success.\n");
    return 0;
  }
  if (fres == FR_EXIST)
    printf(" - already exists.\n");
  else
    printf(" - error.\n");
  return 1;
}


/**************
 * @brief Setup the File System
 * @param state: pointer to the state struct of the System
 * @param gps: pointer to the GPS Frame struct
 * @retval None
*/
void SDFS_SetupFS(struct sState *state, struct sGPSFrame *gps)
{
  state->CreatedFiles = 0;
  state->SubDirectory = 0;

  SDFS_FindNewDirectory(state);
//  snprintf(state->RootPath, 50, "20%02lu%02lu%02lu",
//        (gps->Date) % 100, (gps->Date/100) % 100, (gps->Date/10000));
  //  SDFS_MakeDir(state->RootPath);

  snprintf(state->SpectraPath, 100, "%s/%u",
        state->RootPath, state->SubDirectory);

  SDFS_MakeDir(state->SpectraPath);
}


/**************
 * @brief Make a new Directory based on the last dir name and increment
 * @param state: pointer to the state struct of the System
 * @retval None
*/
void SDFS_IncrementDirectory(struct sState *state)
{
  state->CreatedFiles = 0;
  state->SubDirectory++;

  snprintf(state->SpectraPath, 100, "%s/%u",
        state->RootPath, state->SubDirectory);

  SDFS_MakeDir(state->SpectraPath);
}

/**************
 * @brief Power Cycle the SD Card
 * @param None
 * @retval None
*/
void SDFS_PowerCycle()
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); // power off SD card
  HAL_Delay(150); /* Wait until the capacitors discharge to 0V - Minimum time 80 ms */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // power on SD card
  HAL_Delay(100);
}

/**************
 * @brief Mount the SD Card
 * @param None
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_Mount()
{
  FATFS *pfs;
  FRESULT fres;
  uint32_t fre_clust, fre_sect, tot_sect;

  pfs = &FS;

  if (!BSP_PlatformIsDetected())
  {
    printf("No card detected, aborting SD mount.\n");
    return 1;
  }
  printf("SD card Detected.\n");

  if ((fres = f_mount(&FS, "0:", 1)) != FR_OK)  // mount fs
  {
    printf("Failed to mount the filesystem.\n");
    return 1;
  }
  printf("Filesystem mounted successfully.\n");

  fres = f_getfree("0:", &fre_clust, &pfs); // get free cluster count
  if (fres)
  {
    printf("Failed to read free space.\n");
    return 0; // this technically shouldn't prevent us from using the fs, although it may indicate more inssues
  }
  tot_sect = (pfs->n_fatent - 2) * pfs->csize;
  fre_sect = fre_clust * pfs->csize;

  printf("Filesystem mounted with %lu KiB free out of %lu KiB available.\n", fre_sect / 2, tot_sect / 2);
  return 0;
}


/**************
 * @brief Write the Spectra to the SD Card
 * @param state: pointer to the state struct of the System
 * @param spectra: pointer to the Spectra struct
 * @param gps: pointer to the GPS Frame struct
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_WriteSpectra(struct sState *state, struct sSpectra *spectra, struct sGPSFrame *gps)
{
  uint16_t i, j;
  char fqpn[100];
  FRESULT fres;
  FIL fil;
  char buffer[100];
  unsigned int bytesWritten;

  snprintf(fqpn, 100, "%s/n%05lud-%s.txt", state->SpectraPath, (state->SpectraCount)++, gps->DateTimeStr);

  if ((fres = f_open(&fil, fqpn, FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK)
  {
    printf("Unable to create filename %s, aborting.\n", fqpn);
    return 1;
  }

  sprintf(buffer, "Number of scans: %u\n", spectra->NumberofScans);
  fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);

  sprintf(buffer, "Integration time [us]: %lu\n", spectra->IntegrationTime_us);
  fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);

  j = sizeof spectra->RawData / sizeof spectra->RawData[0];
  j = j - (j%20);

  for (i=14;i<j;i+=20)
  {
    sprintf(buffer, "%u\n%u\n%u\n%u\n%u\n%u\n%u\n%u\n%u\n%u\n",
        (spectra->RawData[i] << 8) | (spectra->RawData[i+1]),
        (spectra->RawData[i+2] << 8) | (spectra->RawData[i+3]),
        (spectra->RawData[i+4] << 8) | (spectra->RawData[i+5]),
        (spectra->RawData[i+6] << 8) | (spectra->RawData[i+7]),
        (spectra->RawData[i+8] << 8) | (spectra->RawData[i+9]),
        (spectra->RawData[i+10] << 8) | (spectra->RawData[i+11]),
        (spectra->RawData[i+12] << 8) | (spectra->RawData[i+13]),
        (spectra->RawData[i+14] << 8) | (spectra->RawData[i+15]),
        (spectra->RawData[i+16] << 8) | (spectra->RawData[i+17]),
        (spectra->RawData[i+18] << 8) | (spectra->RawData[i+19]));

    fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);
  }

  f_close(&fil);
  printf("Completed writing %s\n", fqpn);
  return 0;
}


/**************
 * @brief Write the GPS Data to the SD Card
 * @param state: pointer to the state struct of the System
 * @param buffer: pointer to the buffer containing the string to write
 * @param filename: pointer to the filename to write to
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_WriteString(struct sState *state, uint8_t* buffer, char* filename)
{
  char fqpn[100];
  FRESULT fres;
  FIL fil;
  unsigned int bytesWritten;

  snprintf(fqpn, 100, "%s/%s", state->RootPath, filename);

  if ((fres = f_open(&fil, fqpn, FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK) //////////
  {
    printf("Unable to open/create filename %s, aborting.\n", fqpn);
    return 1;
  }

  fres = f_lseek(&fil, f_size(&fil));

  fres = f_write(&fil, buffer, strlen((char*) buffer), &bytesWritten);

  f_close(&fil);
  return 0;
}


/**************
 * @brief Write the Envoirmental Data to the SD Card
 * @param state: pointer to the state struct of the System
 * @param gps: pointer to the GPS Frame struct
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_WriteEnvironmental(struct sState *state, struct sGPSFrame *gps)
{
  if (gps->Date == 0)
    return 0;

  char fqpn[100];
  FRESULT fres;
  FIL fil;
  char buffer[200];
  unsigned int bytesWritten;

  snprintf(fqpn, 100, "%s/enviro.txt", state->RootPath);

  if ((fres = f_open(&fil, fqpn, FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK) //////////
  {
    printf("Unable to open/create filename %s, aborting.\n", fqpn);
    return 1;
  }

  fres = f_lseek(&fil, f_size(&fil));

  snprintf(buffer, 200, "%s - Temp: %7.2f   Pres: %8.2f  Heater Duty: %3d %3d %3d %3d %3d %3d\n",
      gps->DateTimeStr,
      ((double) state->MSTemperature)/100, ((double) state->MSPressure)/100,
      state->Heaters->HeaterDwell[0], state->Heaters->HeaterDwell[1], state->Heaters->HeaterDwell[2],
      state->Heaters->HeaterDwell[3], state->Heaters->HeaterDwell[4], state->Heaters->HeaterDwell[5]);

  fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);

  f_close(&fil);
//  printf("Completed writing %s\n", fqpn);
  return 0;
}


/**************
 * @brief Write the GPS Data to the SD Card
 * @param state: pointer to the state struct of the System
 * @param gps: pointer to the GPS Frame struct
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_WriteCoords(struct sState *state, struct sGPSFrame *gps)
{
  if (gps->Date == 0)
    return 0;

  char fqpn[200];
  FRESULT fres;
  FIL fil;
  char buffer[200];
  unsigned int bytesWritten;

  snprintf(fqpn, 200, "%s/coords.txt", state->RootPath);

  if ((fres = f_open(&fil, fqpn, FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK) //////////
  {
    printf("Unable to open/create filename %s, aborting.\n", fqpn);
    return 1;
  }

  fres = f_lseek(&fil, f_size(&fil));

  snprintf(buffer, 200, "%s - Lat: %10.6f   Long: %10.6f   Alt: %7.1f  Sats: %02d\n",
      gps->DateTimeStr, gps->Latitude, gps->Longitude, gps->Altitude, gps->NumSats);

  fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);

  f_close(&fil);
//  printf("Completed writing %s\n", fqpn);
  return 0;
}


/**************
 * @brief Write the Spectra Data to the SD Card
 * @param state: pointer to the state struct of the System
 * @param spectra: pointer to the Spectra struct
 * @param gps: pointer to the GPS Frame struct
 * @retval 0 if successful, 1 if not
*/
uint8_t SDFS_WriteSpectraBinary(struct sState *state, struct sSpectra *spectra, struct sGPSFrame *gps)
{
  uint16_t i;
  char fqpn[200];
  FRESULT fres;
  FIL fil;
  unsigned int bytesWritten;

//  snprintf(fqpn, 100, "%s/%s.txt", state->SpectraPath, gps->DateTimeStr);
  snprintf(fqpn, 100, "%s/n%05lud-%s.txt", state->SpectraPath, (state->SpectraCount)++, gps->DateTimeStr);

  if ((fres = f_open(&fil, fqpn, FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) != FR_OK)
  {
    printf("Unable to create filename %s, aborting.\n", fqpn);
    return 1;
  }

  for (i=0;i<8192;i+=512)
    fres = f_write(&fil, &(spectra->RawData[i]), 512, &bytesWritten);

  f_close(&fil);
  printf("Completed writing %s\n", fqpn);
  state->CreatedFiles++;

  return 0;
}

