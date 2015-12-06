/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Drone_Compensator_types.h
 *
 * Code generated for Simulink model 'Drone_Compensator'.
 *
 * Model version                  : 1.3035
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sun Dec  6 03:38:55 2015
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 *    3. ROM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Drone_Compensator_types_h_
#define RTW_HEADER_Drone_Compensator_types_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#ifndef _DEFINED_TYPEDEF_FOR_struct_SNZkZ6rc9vhdEg27lKQdHF_
#define _DEFINED_TYPEDEF_FOR_struct_SNZkZ6rc9vhdEg27lKQdHF_

typedef struct {
  real_T Ts2Q[16];
  real_T Q2Ts[16];
  real_T takeoff_gain;
  real_T totalThrust_maxRelative;
  real_T motorsThrustperMotor_max;
} struct_SNZkZ6rc9vhdEg27lKQdHF;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_pP0yJPvqYhejK9gHgcbWI_
#define _DEFINED_TYPEDEF_FOR_struct_pP0yJPvqYhejK9gHgcbWI_

typedef struct {
  real_T nrotors;
  real_T g;
  real_T rho;
  real_T muv;
  real_T M;
  real_T J[9];
  real_T h;
  real_T d;
  real_T nb;
  real_T r;
  real_T c;
  real_T e;
  real_T Mb;
  real_T Mc;
  real_T ec;
  real_T Ib;
  real_T Ic;
  real_T mb;
  real_T Ir;
  real_T Ct;
  real_T Cq;
  real_T sigma;
  real_T thetat;
  real_T theta0;
  real_T theta1;
  real_T theta75;
  real_T thetai;
  real_T a;
  real_T A;
  real_T gamma;
  real_T b;
  real_T k;
  boolean_T verbose;
} struct_pP0yJPvqYhejK9gHgcbWI;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_27dTQuiR3L9WSuFjHuj2tB_
#define _DEFINED_TYPEDEF_FOR_struct_27dTQuiR3L9WSuFjHuj2tB_

typedef struct {
  real_T batteryStatus[2];
  real_T posVIS_novisionavail[4];
  real_T usePosVIS_flag;
} struct_27dTQuiR3L9WSuFjHuj2tB;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_fgpOXIBQXBGEsNX5uiZ1j_
#define _DEFINED_TYPEDEF_FOR_struct_fgpOXIBQXBGEsNX5uiZ1j_

typedef struct {
  real_T w2ToThrust_gain;
  real_T motors_max;
  real_T motorcommandToW2_gain;
  real_T thrustToMotorcommand;
  real_T noiseStatesSensed_std[12];
  real_T noiseStatesSensed_weights[12];
  real_T sensordelay;
  real_T sensordataCalib[7];
  real_T IMUaccel_gain[3];
  real_T IMUgyro_gain[3];
  real_T airDensity;
  real_T altToPrs_gain;
  real_T inverseIMU_gain[6];
  real_T altSenor_min;
  struct_27dTQuiR3L9WSuFjHuj2tB dummy;
  real_T velocityToOpticalFlow_gain;
  real_T sampletime;
  real_T yawStep_amplitude;
  real_T yawStep_time;
  real_T yawStep_duration;
  real_T pitchStep_amplitude;
  real_T pitchStep_time;
  real_T pitchStep_duration;
  real_T rollStep_amplitude;
  real_T rollStep_time;
  real_T rollStep_duration;
  real_T takeoff_duration;
  real_T altitude;
  real_T NO_VIS_X;
  real_T NO_VIS_YAW;
  real_T motorFailure_time;
  real_T motorFailure_duration;
  real_T motorFailure_m1;
  real_T motorFailure_m2;
  real_T motorFailure_m3;
  real_T motorFailure_m4;
  real_T motorUnknownGain_gain;
} struct_fgpOXIBQXBGEsNX5uiZ1j;

#endif

/* Parameters for system: '<S1>/ControllerPolePlace' */
typedef struct P_ControllerPolePlace_Drone_C_T_ P_ControllerPolePlace_Drone_C_T;

/* Parameters for system: '<Root>/Drone_Compensator' */
typedef struct P_Drone_Compensator_Drone_Com_T_ P_Drone_Compensator_Drone_Com_T;

/* Parameters (auto storage) */
typedef struct P_Drone_Compensator_T_ P_Drone_Compensator_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Drone_Compensator_T RT_MODEL_Drone_Compensator_T;

#endif                                 /* RTW_HEADER_Drone_Compensator_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
