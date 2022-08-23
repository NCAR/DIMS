/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file detumbling.h
* @brief Header of detumbling algorithms.
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
#ifndef LIBADCS_DETUMBLING_H_
#define LIBADCS_DETUMBLING_H_

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* No Includes */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Debug the detumbling process by writing the values to a debug file
#define ADCS_DET_DEBUG                   0
// Detumbling max period (30 mins)
#define ADCS_DET_ALGO_OP_TIME_MAX_MSEC   1800000
// Detumbling algo sampling period (0.5 sec)
#define ADCS_DET_ALGO_SAMPLING_MSEC      ADCS_ALGO_SAMPLING_RATE_MS
// Detumbling stability default
#define ADCS_DET_STABILITY_DEFAULT       500
// Detumbling stability maximum value, before giving up
#define ADCS_DET_STABILITY_MAX           1000
// Detumbling stability error threshold in percent
#define ADCS_DET_STABILITY_ERR_THRESHOLD 20
// When angular velocity is incontrollable, for how long (in ms) to apply rough torque boost (30 min)
#define ADCS_DET_NOCONTROLL_PERIOD_MSEC  1800000

// Detumbling states
// Magnetic field already measured, green light for all estimators
#define ADCS_DETUMB_FIELDB_SET   0x01
// ADCS detumbling bad result
#define ADCS_DETUMB_ALGO_BAD     (~0x02)
// ADCS detumbling good result
#define ADCS_DETUMB_ALGO_GOOD    0x02
// ADCS detumbling - operation complete
#define ADCS_DETUMB_ALGO_RDY     0x04
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Different detumbling algorithms
typedef enum ADCS_Detumbling_Algo_t {
    ADCS_ALGO_BDOT_ES = 0x00,
    ADCS_ALGO_BDOT_ADE,
    ADCS_ALGO_DTORQLINEAR,
    ADCS_ALGO_DTORQCUBIC
} ADCS_Detumbling_Algo_t;
// States are function of state period (t) = stateNo * ADCS_DET_ALGO_SAMPLING_MSEC.
typedef enum ADCS_Detumbling_Algo_State_t {
    ADCS_DET_ALGO_STOP_MTORQ   = 0, // Stop the magnetorquers
    ADCS_DET_ALGO_MEASURE_MAG  = 5, // Measure the magnetic field. This value is directly related to the magnetorquers dead time
    ADCS_DET_ALGO_EXECUTE_TORQ = 6, // Measure the magnetic field and gyros (latter - if applicable) and start applying the magnetic torque
    ADCS_DET_ALGO_RESTART      = 12 // Restart the algo
} ADCS_Detumbling_Algo_State_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
//extern volatile uint8_t ADCSDetState;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern void Detumbling_Init (uint8_t gyroUsage, REAL magnetoGain, REAL gyroGain);
extern uint8_t Detumbling_Routine (ADCS_Detumbling_Algo_t algoType, uint8_t negVector);
extern void Detumbling_SetState (uint8_t dState);
extern uint8_t Detumbling_GetState (void);
extern uint8_t Detumbling_GetQuarantineTick (void);
extern void ADCS_Detumbling_GetStatus (ADCS_Status_t *status);

#endif    /* LIBADCS_DETUMBLING_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
