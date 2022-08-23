/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ADCS_bindings.h
* @brief Header of ADCS hardware bindings.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Georgi Georgiev
* @version           1.0.0
* @date              2019.09.05
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.09.05, author Georgi Georgiev, Initial revision }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef LIBADCS_ADCS_BINDINGS_H_
#define LIBADCS_ADCS_BINDINGS_H_

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "MCU_Init.h"
#include "DAT_Inputs.h"
#include "panels.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Debug the ADCS process with simulated data
#define ADCS_SIM_DEBUG                   0
// Whether ADCS algo will be governed by RTOS or directly by timer and polling task
#define ADCS_RTOS_ENABLED                1
// Battery check enabled
#define ADCS_BATTERY_CHECK_ENABLED       1
// Battery hysteresis thresholds
#define ADCS_BATTERY_HIST_LOW            0x658 // 3.8V
#define ADCS_BATTERY_HIST_HIGH           0x6AD // 4.0V => 4 V / 0.0023394775 = 1709 ADC
// Whether to use internal rad/s conversion for gyro input data or not
#define ADCS_GYRO_ANGLE_FILTER           1
// Debug simulation macros
#if ADCS_SIM_DEBUG
    #define ADCS_SIM_RAND(x)             ((REAL)rand() / ((REAL)RAND_MAX / ((x) * 2)) - (x))
#endif
// Floating number storage type
#define REAL                             double
// Global return flags
#define ADCS_OK                          0x00
#define ADCS_ERR                         0x01
// Magnetometer bindings
#define ADCS_MAG_ADDRESS_LOW             LIS3MDL_MAG_I2C_ADDRESS_LOW
#define ADCS_MAG_ADDRESS_HIGH            LIS3MDL_MAG_I2C_ADDRESS_HIGH
// Photo panels
#define ADCS_PHOTO_SENSORQTY             MAX_PAN
// Debug data from file or analytic
// Invalid sensor ID / All sensor IDs
#define ADCS_ALL_SENSORS_ID              0xFF
// Conversion macros
#define CONV_AXIS2VEC(A,V)               { V.X.Axis = A.AXIS_X; V.Y.Axis = A.AXIS_Y; V.Z.Axis = A.AXIS_Z; }
#define CONV_VEC2AXIS(A,V)               { A.AXIS_X = V.X.Axis; A.AXIS_Y = V.Y.Axis; A.AXIS_Z = V.Z.Axis; }
#define CONV_TEMP2VEC(A,V)               { V.X.Temp = A.Temp_X; V.Y.Temp = A.Temp_Y; V.Z.Temp = A.Temp_Z; }
#define CONV_VEC2TEMP(A,V)               { A.Temp_X = V.X.Temp; A.Temp_Y = V.Y.Temp; A.Temp_Z = V.Z.Temp; }
// All algorithms sampling rate (in ms)
#define ADCS_ALGO_SAMPLING_RATE_MS      500
// Check batter status rate (in ms)
#define ADCS_BAT_SAMPLING_RATE_MS       30000
// ADCS Status state bits
#define ADCS_STATUS_TGTEN               0x0001
#define ADCS_STATUS_LOWBAT              0x0002

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef union Vec3D_Axis_t {
    REAL Temp;
    REAL Axis;
} Vec3D_Axis_t;
typedef struct Vec3D_t {
    union {
        struct {
            Vec3D_Axis_t X, Y, Z;
        };
        Vec3D_Axis_t Vec[3];
    };
} Vec3D_t;
typedef enum Vec3D_Gyro_Types_t {
    VEC3D_GYRO_RATE = 0x00,
    VEC3D_GYRO_ANGLE,
    VEC3D_GYRO_TEMP,
    VEC3D_GYRO_ALL
} Vec3D_Gyro_Types_t;
typedef enum Vec3D_Photo_Types_t {
    VEC3D_PHOTO_LIGHT = 0x00,
    VEC3D_PHOTO_TEMP
} Vec3D_Photo_Types_t;
typedef enum Vec3D_Sensor_Ctrl_t {
    VEC3D_SENSOR_STOP = 0x00,
    VEC3D_SENSOR_START
} Vec3D_Sensor_Ctrl_t;
typedef enum ADCS_UUID_t {
    ADCS_UUID_NOMATCH     = 0x00,
    ADCS_UUID_MATCH       = 0x01,
    ADCS_UUID_UNSUPPORTED = 0xFF
} ADCS_UUID_t;
typedef struct ADCS_Status_t {
    uint8_t systemEnabled;
    uint8_t algorithm[2];
    uint8_t gyrosEnabled;
    int32_t stability;
    uint16_t state;
    uint8_t flags;
    uint32_t qTick;
    uint32_t magErrors, gyroErrors;
} ADCS_Status_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* No External variables declarations */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern void ADCS_Bindings_Init(void);
extern ADCS_UUID_t ADCS_CheckUID(void);
extern uint8_t ADCS_Ctrl_Magneto (uint8_t magnetoID, Vec3D_Sensor_Ctrl_t action);
extern uint8_t ADCS_Get_Magneto (uint8_t magnetoID, Vec3D_t *vec);
extern void ADCS_Get_MagnetoFiltered (Vec3D_t *currVec, Vec3D_t *prevVec, Vec3D_t *destVec);
extern uint8_t ADCS_Ctrl_Gyro (uint8_t gyroID, Vec3D_Sensor_Ctrl_t action);
extern uint8_t ADCS_Get_Gyro (uint8_t gyroID, Vec3D_Gyro_Types_t vecType, Vec3D_t *vec);
extern uint8_t ADCS_Ctrl_Photo (uint8_t panelID, Vec3D_Sensor_Ctrl_t action);
extern uint8_t ADCS_Get_Photo (uint8_t panelID, Vec3D_Photo_Types_t vecType, uint16_t *photoData);
extern uint8_t ADCS_Ctrl_Torque (uint8_t torqueID, Vec3D_Sensor_Ctrl_t action);
#if ADCS_SIM_DEBUG
    extern void AMBER_LED_STROBE (uint32_t ms);
    extern void AMBER_LED_SOS(void);
    extern void AMBER_LED_SOS_INFINITE(void);
    extern void AMBER_LED_STROBE_BIT (uint8_t b);
    extern void AMBER_LED_STROBE_ERRNO_8BIT (uint8_t n);
    extern uint8_t ADCS_Get_Torque (Vec3D_t *sensorGain, Vec3D_t *vec);
#else
    #define AMBER_LED_STROBE(ms)
    #define AMBER_LED_SOS()
    #define AMBER_LED_SOS_INFINITE()
    #define AMBER_LED_STROBE_BIT(b)
    #define AMBER_LED_STROBE_ERRNO_8BIT(n)
    extern uint8_t ADCS_Set_Torque (uint8_t torqueID, Vec3D_t *vec);
    extern uint8_t ADCS_Boost_Torques (void);
    extern void ADCS_Stop_Torques (void);
#endif /* ADCS_SIM_DEBUG */
extern uint8_t ADCSGetRTC (uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec);
extern int8_t ADCSCheckRTC (uint64_t reducedUTCDate);
extern void ADCSDelayMs (uint32_t ms);
extern uint32_t ADCSGetSysTick (void);
extern uint8_t ADCS_Battery_Low (void);

#endif    /* LIBADCS_ADCS_BINDINGS_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

