/*
 * GPS.c
 *  This file contains the functions to process the GPS data
 *  Created on: Apr 21, 2021
 *      Author: damonb
 *  Edited by mjeffers
 */

#include "gps.h"
#include <stdlib.h>
#include <math.h>
#include "funcs.h"


/*******
 * @brief This will Initialize the GPS Frame
 * @param gps: pointer to the GPS Frame struct
 * @retval None
*/
void InitGPSFrame(struct sGPSFrame* gps)
{
//  gps->TimeStr = 0;
//  gps->DateStr = 0;
//  gps->LongStr = 0;
//  gps->LatStr = 0;
  gps->RMCTimeStr[0] = 0;
  gps->GGATimeStr[0] = 0;
  gps->DateStr[0] = 0;
  gps->LongStr[0] = 0;
  gps->LatStr[0] = 0;
  gps->AltStr[0] = 0;
  gps->NS='n';
  gps->EW='n';
  gps->Time = 0;
  gps->Date = 0;
  gps->Longitude = 0;
  gps->Latitude = 0;
  gps->Fixed = false;
  gps->Ticks = 0;
  gps->Status = 0;
  gps->Altitude = 0;
  gps->NumSats = 0;
  gps->DateTimeStr[0] = 0;
}


/*********
 * @brief This will parse the GPS data
 * @param gps: pointer to the GPS Frame struct
 * @retval 0 if successful, 1 if not
*/
uint8_t ProcessGPSFrame(struct sGPSFrame* gps)
{
  double val, deg, min;
  int i;
  char* RMCFields[20];
  char* GGAFields[20];
  uint8_t rs[90] = {0};
  uint8_t gs[90] = {0};
  strncpy((char*) rs, gps->RMCSentence, 90);
  strncpy((char*) gs, gps->GGASentence, 90);

  parse_comma_delimited_str((char*) rs, RMCFields, 20);
  parse_comma_delimited_str((char*) gs, GGAFields, 20);
  if (strncmp(RMCFields[1], GGAFields[1], 6) != 0)  // if header does not match
    return 1;
  /* $GxRMC
   * $GNRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
   * ex: $GPRMC,230558.501,A,4543.8901,N,02112.7219,E,1.50,181.47,230213,,,A*66,
   *
   * WORDS:
   *  0    = "GxRMC"
   *  1    = UTC of position fix
   *  2    = Data status (V=navigation receiver warning)
   *  3    = Latitude of fix
   *  4    = N or S
   *  5    = Longitude of fix
   *  6    = E or W
   *  7    = Speed over ground in knots
   *  8    = Track made good in degrees True, Bearing This indicates the direction that the device is currently moving in,
   *       from 0 to 360, measured in “azimuth”.
   *  9    = UT date
   *  10   = Magnetic variation degrees (Easterly var. subtracts from true course)
   *  11   = E or W
   *  12   = Checksum
   */


  /* $GxGGA
   * $GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
   * ex: $GPGGA,230600.501,4543.8895,N,02112.7238,E,1,03,3.3,96.7,M,39.0,M,,0000*6A,
   *
   * WORDS:
   *  0    = "GxGGA"
   *  1    = UTC of Position
   *  2    = Latitude
   *  3    = N or S
   *  4    = Longitude
   *  5    = E or W
   *  6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
   *  7    = Number of satellites in use [not those in view]
   *  8    = Horizontal dilution of position
   *  9    = Antenna altitude above/below mean sea level (geoid)
   *  10   = Meters  (Antenna height unit)
   *  11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level.
   *      -geoid is below WGS-84 ellipsoid)
   *  12   = Meters  (Units of geoidal separation)
   *  13   = Age in seconds since last update from diff. reference station
   *  14   = Diff. reference station ID#
   *  15   = Checksum
   */


  gps->Status = 0;
  if (GGAFields[6] > 0)
    gps->Fixed = true;

  strncpy(gps->RMCTimeStr, RMCFields[1], 20);
  strncpy(gps->GGATimeStr, GGAFields[1], 20);
  strncpy(gps->DateStr,    RMCFields[9], 20);
  strncpy(gps->LongStr,    RMCFields[5], 20);
  strncpy(gps->LatStr,     RMCFields[3], 20);
  strncpy(gps->AltStr,     GGAFields[9], 20);

  gps->NS = RMCFields[4][0];
  gps->EW = RMCFields[6][0];
  gps->Time = strtol(gps->RMCTimeStr, NULL, 10);
  gps->Date = strtol(gps->DateStr, NULL, 10);

  val = strtof(gps->LatStr, NULL) / 100;
  deg = floor(val);
  min = (val-deg) * 100;
  if (gps->NS == 'N')
    gps->Status |= (1<<24);
  gps->Latitude = deg + min/60;

  val = strtof(gps->LongStr, NULL) / 100;
  deg = floor(val);
  min = (val-deg) * 100;
  if (gps->EW == 'E')
    gps->Status |= (1<<25);
  gps->Longitude = deg + min/60;

  val = strtof(gps->AltStr, NULL);
  gps->Altitude = val;

  gps->NumSats = strtol(GGAFields[7], NULL, 10);

  i = strtol(GGAFields[6], NULL, 10);
  switch (i)
  {
    case 0:
      break;
    case 1:
      gps->Status |= (1<<26);
      break;
    case 2:
      gps->Status |= (1<<27);
      break;
    case 6:
      gps->Status |= (1<<28);
      break;
  }

  gps->Ticks = 0;
//  snprintf(gps->DateTimeStr, 100, "20%02lu-%02lu-%02lu %02lu:%02lu:%02lu",
  snprintf(gps->DateTimeStr, 100, "20%02lu%02lu%02lu %02lu%02lu%02lu",
      (gps->Date) % 100, (gps->Date/100) % 100, (gps->Date/10000),
      (gps->Time/10000), (gps->Time/100) % 100, (gps->Time) % 100);
  return 0;
}

