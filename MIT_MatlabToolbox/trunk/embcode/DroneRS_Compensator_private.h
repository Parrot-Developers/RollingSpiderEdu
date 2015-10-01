/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DroneRS_Compensator_private.h
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

#ifndef RTW_HEADER_DroneRS_Compensator_private_h_
#define RTW_HEADER_DroneRS_Compensator_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               (&(rtm)->Timing.taskTime0)
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void DroneRS_Co_ControllerPID2W_Init(DW_ControllerPID2W_DroneRS_Co_T
  *localDW, P_ControllerPID2W_DroneRS_Com_T *localP);
extern void DroneRS_C_ControllerPID2W_Start(RT_MODEL_DroneRS_Compensator_T *
  const DroneRS_Compensator_M, DW_ControllerPID2W_DroneRS_Co_T *localDW);
extern void DroneRS_Compens_ControllerPID2W(const real_T rtu_pos_ref[3], const
  real_T rtu_att_ref[3], boolean_T rtu_controlModePosVatt_flagin, const real_T
  rtu_states_estim[3], const real_T rtu_states_estim_c[3], real_T
  rtu_states_estim_l, real_T rtu_states_estim_h, const real_T
  rtu_states_estim_e[2], const real_T rtu_states_estim_d[2],
  B_ControllerPID2W_DroneRS_Com_T *localB, DW_ControllerPID2W_DroneRS_Co_T
  *localDW, P_ControllerPID2W_DroneRS_Com_T *localP, P_DroneRS_Compensator_T
  *DroneRS_Compensator_P);
extern void DroneRS_Compe_MeasurementUpdate(boolean_T rtu_Enable, const real_T
  rtu_Lk[4], const real_T rtu_yk[2], const real_T rtu_yhatkk1[2],
  B_MeasurementUpdate_DroneRS_C_T *localB);
extern void DroneRS_Com_UseCurrentEstimator(boolean_T rtu_Enablek, const real_T
  rtu_Mk[4], const real_T rtu_uk[2], const real_T rtu_yk[2], const real_T
  rtu_Ck[4], const real_T rtu_Dk[4], const real_T rtu_xhatkk1[2],
  B_UseCurrentEstimator_DroneRS_T *localB);
extern void DroneR_DroneRS_Compensator_Init(DW_DroneRS_Compensator_DroneR_T
  *localDW, P_DroneRS_Compensator_DroneRS_T *localP);
extern void Drone_DroneRS_Compensator_Start(RT_MODEL_DroneRS_Compensator_T *
  const DroneRS_Compensator_M, DW_DroneRS_Compensator_DroneR_T *localDW);
extern void DroneRS_Com_DroneRS_Compensator(boolean_T
  rtu_controlModePosVSAtt_flagin, const real_T rtu_pos_refin[3], const real_T
  rtu_attRS_refin[3], real_T rtu_sensordataRS_datin, real_T
  rtu_sensordataRS_datin_j, real_T rtu_sensordataRS_datin_n, real_T
  rtu_sensordataRS_datin_jr, real_T rtu_sensordataRS_datin_k, real_T
  rtu_sensordataRS_datin_c, real_T rtu_sensordataRS_datin_g, real_T
  rtu_sensordataRS_datin_gz, const real_T rtu_opticalFlowRS_datin[3], const
  real_T rtu_sensordatabiasRS_datin[7], const real_T rtu_posVIS_datin[4], real_T
  rtu_usePosVIS_flagin, const real_T rtu_batteryStatus_datin[2],
  B_DroneRS_Compensator_DroneRS_T *localB, DW_DroneRS_Compensator_DroneR_T
  *localDW, P_DroneRS_Compensator_DroneRS_T *localP, P_DroneRS_Compensator_T
  *DroneRS_Compensator_P);

#endif                                 /* RTW_HEADER_DroneRS_Compensator_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
