/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ADCSMathPhys.h
* @brief Header of ADCSMathPhys.h.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Viktor D., Georgi Georgiev
* @version           1.0.0
* @date              2019.10.02
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.10.02, author Georgi Georgiev, Initial revision }
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef LIBADCS_ADCSMATHPHYS_H_
#define LIBADCS_ADCSMATHPHYS_H_

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
/* No External defines*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef enum RefFrame_t {
    ADCSMP_FIXED2INERTIAL,
    ADCSMP_INERTIAL2FIXED
} RefFrame_t;
typedef enum AxisDev_t {
    ADCSMP_AXISDEV_NEAR_PERFECT = 0,
    ADCSMP_AXISDEV_SMALL_ENOGUH = 1,
    ADCSMP_AXISDEV_TOO_LARGE = 5
} AxisDev_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern const REAL cPi, cRe, cReKM, cM, cG, cMu, cJ2, cRhoA, cPhiPole, cThPole;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
extern void ADCSMathPhys_Init (void);
extern REAL ADCSMP_MatDet (Vec3D_t *A);
extern REAL ADCSMP_MatTrace (Vec3D_t *A);
extern void ADCSMP_MatMult (Vec3D_t *A, Vec3D_t *B, Vec3D_t *C);
extern void ADCSMP_MatMultVec (Vec3D_t *A, Vec3D_t B, Vec3D_t *C);
extern REAL ADCSMP_VecScalarProduct (Vec3D_t v, Vec3D_t w);
extern REAL ADCSMP_VecNorm (Vec3D_t v);
extern void ADCSMP_VecNormalize (Vec3D_t *v);
extern void ADCSMP_VecProduct (Vec3D_t v, Vec3D_t w, Vec3D_t *u);
extern REAL ADCSMP_VecAngle (Vec3D_t v, Vec3D_t w);
extern void ADCSMP_MatInverse (Vec3D_t *A, Vec3D_t *B);
extern REAL ADCSMP_JulianDate (REAL YY, REAL MM, REAL DD, REAL H, REAL M, REAL S);
extern void ADCSMP_Cartesian2Kepler (Vec3D_t R, Vec3D_t V, Vec3D_t *K1, Vec3D_t *K2);
extern void ADCSMP_SunPos (REAL JD, Vec3D_t *Spos, uint8_t CoordType);
extern void ADCSMP_ForceG (Vec3D_t R, Vec3D_t V, Vec3D_t *F, REAL J2, REAL K);
extern void ADCSMP_TransformRefFrame (RefFrame_t refFrame, REAL JD, Vec3D_t *rI, Vec3D_t *rF);
extern void ADCSMP_DipMagField (REAL JD, Vec3D_t Rin, Vec3D_t *B);
extern void ADCSMP_MatToEulerAng (Vec3D_t *A, Vec3D_t *E);
extern void ADCSMP_MatToAxis (Vec3D_t *A, Vec3D_t *E, REAL *Phi);
extern AxisDev_t ADCSMP_AxisToMu (Vec3D_t E, Vec3D_t B, REAL allowedAngularDiff, Vec3D_t *Mu);
extern void ADCSMP_MagFieldV1 (REAL Th, REAL Phi, REAL altitude, Vec3D_t *B);
extern void ADCSMP_SunPosV1 (REAL JD, Vec3D_t *B);
extern void ADCSMP_MoonPosV1 (REAL JD, Vec3D_t *B);
extern void ADCSMP_SunDirectionV1 (REAL maxPower, REAL noiseMax, REAL albedoLevel, uint16_t *photoSensorInput, Vec3D_t *sunDir);

#endif    /* LIBADCS_ADCSMATHPHYS_H_ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
