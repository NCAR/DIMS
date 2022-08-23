/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file detumblingInternal.h
* @brief Header of the internal detumbling algorithms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Georgi Georgiev
* @version           1.0.0
* @date              2019.12.04
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.12.04, author Georgi Georgiev, Initial revision }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef LIBADCS_DETUMBLING_INTERNAL_H_
#define LIBADCS_DETUMBLING_INTERNAL_H_

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
// Magnetic field at step X - 1
#define ADCS_FIELD_PREV         0x00
// Magnetic field at step X
#define ADCS_FIELD_CURR         0x01
// Magnetic field cache array size
#define ADCS_FIELDB_ALL         0x02
// Algo estimator at step X - 1
#define ADCS_ESTIM_PREV         0x00
// Algo estimator at step X
#define ADCS_ESTIM_CURR         0x01
// Algo estimator result at step X - 1
#define ADCS_ESTIM_RES_PREV     0x02
// Algo estimator result at step X
#define ADCS_ESTIM_RES_CURR     0x03
// Algo estimator cache array size
#define ADCS_ESTIM_ALL          0x04

// Conversion from T to gauss
#define ADCS_CONV_T2GAUSS(x)    ((x) / 230.0F)
// Conversion from gauss to T
#define ADCS_CONV_GAUSS2T(x)    ((x) * 230.0F)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* No external type declarations */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* No external variables declarations */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern void Detumbling_DTorqLinearGyroFeedback_Estimator (Vec3D_t gyroData[VEC3D_GYRO_ALL], Vec3D_t fieldB[ADCS_FIELDB_ALL], Vec3D_t algoEstimation[ADCS_ESTIM_ALL]);
extern void Detumbling_DTorqCubicGyroFeedback_Estimator (Vec3D_t gyroData[VEC3D_GYRO_ALL], Vec3D_t fieldB[ADCS_FIELDB_ALL], Vec3D_t algoEstimation[ADCS_ESTIM_ALL], REAL magGain, REAL gGain);


#endif    /* LIBADCS_DETUMBLING_INTERNAL_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
