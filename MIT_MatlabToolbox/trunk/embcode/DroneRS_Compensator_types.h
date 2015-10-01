/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DroneRS_Compensator_types.h
 *
 * Code generated for Simulink model 'DroneRS_Compensator'.
 *
 * Model version                  : 1.2576
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Oct  1 18:12:10 2015
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

#ifndef RTW_HEADER_DroneRS_Compensator_types_h_
#define RTW_HEADER_DroneRS_Compensator_types_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
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

#ifndef _DEFINED_TYPEDEF_FOR_struct_eTOByJ6BrrCe8gZfBpKFUD_
#define _DEFINED_TYPEDEF_FOR_struct_eTOByJ6BrrCe8gZfBpKFUD_

typedef struct {
  real_T filter_b_gyroz[6];
  real_T filter_a_gyroz[6];
  real_T filter_b_prs[6];
  real_T filter_a_prs[6];
  real_T outlierJump_UpperLimit;
  real_T stateDeviationPrs_Threshold;
  real_T stateDeviationSonflt_Threshold;
} struct_eTOByJ6BrrCe8gZfBpKFUD;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_nVjCgugzLFJzCZr6yyeDeH_
#define _DEFINED_TYPEDEF_FOR_struct_nVjCgugzLFJzCZr6yyeDeH_

typedef struct {
  real_T motorsRSToW2_Gain;
  real_T noiseStatesSensed_std[12];
  real_T noiseStatesSensed_weights[12];
  real_T sensordataRSbias[7];
  real_T accelo_Gain[3];
  real_T gyropq_Gain[2];
  real_T gyror_Gain;
  real_T airDensity;
  real_T altToPrs_Gain;
  real_T VelocityToOpticalFlow_Gain;
  real_T inversesIMU_Gain[6];
  real_T altSenor_LowerLimit;
  real_T sampleTime_qcsim;
  real_T SIMDUMMYposVIS_noVisionFlag[4];
  real_T SIMDUMMYusePosVIS_flag;
  real_T motorFailure_time;
  real_T motorFailure_duration;
  real_T motorFailure_gainm2;
  real_T motorFailure_gainm3;
} struct_nVjCgugzLFJzCZr6yyeDeH;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_rM3FFntOU5Aaym8djgtmlC_
#define _DEFINED_TYPEDEF_FOR_struct_rM3FFntOU5Aaym8djgtmlC_

typedef struct {
  real_T pitchroll_UpperLimit;
  real_T pq_UpperLimit;
  real_T pq_UpperLimit_hov;
  real_T dpq_UpperLimit;
  real_T Z_UpperLimit;
  real_T deltadxy_UpperLimit;
} struct_rM3FFntOU5Aaym8djgtmlC;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_YkbJnRR8M5ye4XtO88GdQC_
#define _DEFINED_TYPEDEF_FOR_struct_YkbJnRR8M5ye4XtO88GdQC_

typedef struct {
  real_T att_UpperLimit;
  real_T deltaXY;
} struct_YkbJnRR8M5ye4XtO88GdQC;

#endif

/* Parameters for system: '<S1>/ControllerPID2W' */
typedef struct P_ControllerPID2W_DroneRS_Com_T_ P_ControllerPID2W_DroneRS_Com_T;

/* Parameters for system: '<Root>/DroneRS_Compensator' */
typedef struct P_DroneRS_Compensator_DroneRS_T_ P_DroneRS_Compensator_DroneRS_T;

/* Parameters (auto storage) */
typedef struct P_DroneRS_Compensator_T_ P_DroneRS_Compensator_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_DroneRS_Compensator_T RT_MODEL_DroneRS_Compensator_T;

#endif                                 /* RTW_HEADER_DroneRS_Compensator_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
