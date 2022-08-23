/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file control.h
* @brief Header of ADCS control machinery.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Georgi Georgiev
* @version           1.0.0
* @date              2019.10.18
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.10.18, author Georgi Georgiev, Initial revision }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef LIBADCS_CONTROL_H_
#define LIBADCS_CONTROL_H_

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
// Debug the Control process by writing the values to a debug file
#define ADCS_CTRL_DEBUG                          0
// Debug the Control process by simulation
#define ADCS_CTRL_SIM                            0
// Control propagation intervals (in ms)
#define ADCS_CTRL_PROP_TIME                      30000
// Control algo sampling period (0.5 sec)
#define ADCS_CTRL_ALGO_SAMPLING_MSEC             ADCS_ALGO_SAMPLING_RATE_MS
// Control stability default
#define ADCS_CTRL_STABILITY_DEFAULT              500
// Control stability maximum value, before giving up
#define ADCS_CTRL_STABILITY_MAX                  1000
// Control stability error threshold in percent
#define ADCS_CTRL_STABILITY_ERR_THRESHOLD        20

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Different control algorithms
typedef enum ADCS_Ctrl_Algo_t {
    ADCS_ALGO_CTRL_NONE = 0x00,
    ADCS_ALGO_CTRL_PID_ES
} ADCS_Ctrl_Algo_t;
// Satellite's targeting wall, upon manual targeting
typedef enum ADCS_TargetingWall_t {
    ADCS_TGTWALL_X = 0x00,
    ADCS_TGTWALL_Y,
    ADCS_TGTWALL_Z
} ADCS_TargetingWall_t;
// States are function of state period (t) = stateNo * ADCS_CTRL_ALGO_SAMPLING_MSEC.
typedef enum ADCS_Determination_Algo_State_t {
    ADCS_CTRL_ALGO_START     = 0, // Start the control (stopped magnetorquers)
    ADCS_CTRL_ALGO_STOP_TORQ = 6, // Stop the magnetorquers
    ADCS_CTRL_ALGO_RESET     = 12 // Restar the algo
} ADCS_Determination_Algo_State_t;
// Position targeting structure
typedef struct ADCS_Target_t {
    uint8_t targetPosIsECF, targetingWall;
    Vec3D_t targetPosECFECI;
} ADCS_Target_t;

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
extern void Control_Init (ADCS_Ctrl_Algo_t ctrlAlgo, uint8_t gyroUsage, REAL allowedAngularDiff);
extern uint8_t Control_Routine (ADCS_Target_t *target);
extern void ADCS_DetCtrl_GetStatus (ADCS_Status_t *status);

#endif    /* LIBADCS_CONTROL_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
