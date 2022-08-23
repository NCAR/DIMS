/*
 * GPS.h
 *
 *  Created on: Apr 21, 2021
 *      Author: damonb
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "defs.h"

struct sGPSFrame
{
  char RMCSentence[90];
  char GGASentence[90];
  char RMCTimeStr[20];
  char GGATimeStr[20];
  char Quality;
  char DateStr[20];
  char LongStr[20];
  char LatStr[20];
  char AltStr[20];
  char NS;
  char EW;
  uint32_t Ticks;
  uint32_t Time;
  uint32_t Date;
  double Longitude;  // decimal degrees
  double Latitude;  // decimal degrees
  double Altitude; // meters
  bool Fixed;
  uint8_t NumSats;
  uint32_t Status;
  char DateTimeStr[100];

};


void InitGPSFrame(struct sGPSFrame* gps);
uint8_t ProcessGPSFrame(struct sGPSFrame* gps);




#endif /* INC_GPS_H_ */
