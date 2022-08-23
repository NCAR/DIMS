/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ADCS.h
* @brief Header of ADCS.
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
#ifndef LIBADCS_ADCS_H_
#define LIBADCS_ADCS_H_

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include <libADCS/ADCS_bindings.h>
#include <libADCS/ADCSMathPhys.h>
#include <libADCS/detumbling.h>
#include <libADCS/control.h>
#include <libADCS/determination.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* No external defines */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// ADCS generic states
typedef enum ADCS_state_t {
    ADCS_STATE_PREDEPLOYMENT = 0x00,
    ADCS_STATE_DETUMBLING,
    ADCS_STATE_ADCS,
    ADCS_STATE_TARGETING_POS,
    ADCS_STATE_INVALID = 0xFF
} ADCS_state_t;
// ADCS gyroscope usage
typedef enum ADCS_GyroUsage_t {
    ADCS_GYROUSAGE_NONE = 0x00,
    ADCS_GYROUSAGE_DETUMBLING = 0x01,
    ADCS_GYROUSAGE_CONTROL = 0x02
} ADCS_GyroUsage_t;
/* --== ADCS System configuration ==--
 * <<< Available detumbling algorithms >>>
 * ADCS_ALGO_BDOT_ES - B-dot by PD estimator (by Paurov)
 * ADCS_ALGO_BDOT_ADE - B-dot by Andersen digital estimator
 * ADCS_ALGO_DTORQLINEAR - Desired torque with linear gyroscope feedback (w/ gyro only)
 * ADCS_ALGO_DTORQCUBIC - Desired torque with cubic gyroscope feedback (w/ gyro only)
 * <<< Available determination algorithms >>>
 * ADCS_ALGO_TRIAD_ES - TRIAD EnduroSat implementation by Viktor Danchev
 * <<< Available control algorithms >>
 * ADCS_ALGO_CTRL_NONE - Pure proportional controller implementation by Viktor Danchev
 * ADCS_ALGO_CTRL_PID_ES - TRIAD EnduroSat implementation by Viktor Danchev
 */
typedef struct ADCS_Config_t {
    ADCS_Detumbling_Algo_t detumblingAlgo;
    ADCS_Determination_Algo_t determinationAlgo;
    ADCS_Ctrl_Algo_t controlAlgo;
    ADCS_DetConfig_t determinationConfig;
    REAL magnetoGain, gyroGain, allowedAngularDiff;
    uint64_t startDetumblingAt; // YYMMDDHHMMSS0000
    uint8_t configMask;
} ADCS_Config_t;

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
extern void ADCS_Init (ADCS_Config_t *icfg);
extern uint8_t ADCS_SetState (ADCS_state_t dState, void *data, uint8_t force);
extern ADCS_state_t ADCS_GetState (void);
extern ADCS_Config_t *ADCS_GetConfig (void);
extern uint8_t ADCS_GetStatus (ADCS_Status_t *status);

#endif    /* LIBADCS_ADCS_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
