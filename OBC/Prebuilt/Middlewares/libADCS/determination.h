/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file determination.h
* @brief Header of ADCS determination machinery.
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
#ifndef LIBADCS_DETERMINATION_H_
#define LIBADCS_DETERMINATION_H_

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
// Debug the determination process by writing the values to a debug file
#define ADCS_DTR_DEBUG                          0
// Debug the determination process by utilization of unit tests
#define ADCS_DTR_UNIT_TESTS                     0
// Determination algo sampling period (0.5 sec)
#define ADCS_DTR_ALGO_SAMPLING_MSEC             ADCS_ALGO_SAMPLING_RATE_MS
// Critical linear regression correlation coefficient threshold - should be between 0 (not a real world case) and growing exponentially to higher error rates (0.6 - 0.9 - real world range)
#define ADCS_DTR_LINREG_CORELCOEFF_THRESHOLD    0.7
// Proportional execution - Propagator : Linear regression
#define ADCS_LTR_EXEC_PROPAGATOR                3
#define ADCS_LTR_EXEC_LINREG                    7
// Determination algorithm propagation time (in ms)
#define ADCS_DTR_PROP_TIME                      (ADCS_LTR_EXEC_PROPAGATOR * ADCS_DTR_ALGO_SAMPLING_MSEC)
// Utilize gyro data
#define ADCS_DTR_GYRO_ENABLE                    0

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Different determination algorithms
typedef enum ADCS_Determination_Algo_t {
    ADCS_ALGO_TRIAD_ES = 0x00
} ADCS_Determination_Algo_t;
// Photo sensor configuration types
typedef enum ADCSDetPhotoSensorConfig_t {
    ADCS_DET_PHOTO_VMAX = 0x00,
    ADCS_DET_PHOTO_NOISEMAX,
    ADCS_DET_PHOTO_ALBEDO,
    ADCS_DET_PHOTO_ALLCONFIG
} ADCSDetPhotoSensorConfig_t;
// Satellite initial parametrization for determination
typedef struct ADCS_DetConfig_t {
    Vec3D_t iniPosition;
    Vec3D_t iniVelocity;
    REAL satMass;
    REAL satDragCoeff;
    REAL satAvgDragArea;
    REAL photoSensorConfig[ADCS_DET_PHOTO_ALLCONFIG];
} ADCS_DetConfig_t;

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
extern void Determination_Init (ADCS_Determination_Algo_t orbitalAlgo, ADCS_DetConfig_t detC);
extern void Determination_GC_Pos (void);
#if !ADCS_CTRL_SIM
extern uint8_t Determination_Estimator (REAL *JD, Vec3D_t *satPos, Vec3D_t *attMatrix, Vec3D_t *axisVec, REAL *Phi, Vec3D_t *magVec, uint8_t *magErrorDetected);
#else
extern uint8_t Determination_Estimator (Vec3D_t *attMatrix, Vec3D_t *axisVec, REAL *Phi, Vec3D_t *magVec, Vec3D_t *w, Vec3D_t *optMu);
extern void Determination_LogTorque (int32_t stability, Vec3D_t torq);
#endif /* !ADCS_CTRL_SIM */
#if ADCS_DTR_UNIT_TESTS
extern uint8_t Determination_UnitTests (void);
#endif /* ADCS_DTR_UNIT_TESTS */

#endif    /* LIBADCS_DETERMINATION_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
