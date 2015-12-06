/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Drone_Compensator_private.h
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

#ifndef RTW_HEADER_Drone_Compensator_private_h_
#define RTW_HEADER_Drone_Compensator_private_h_
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

extern void Drone_Compe_ControllerPolePlace(const real_T rtu_pos_refin[3],
  boolean_T rtu_takeoff_flag, const real_T rtu_orient_refin[3], boolean_T
  rtu_controlModePosVSOrient_flag, const real_T rtu_states_estimin[2], const
  real_T rtu_states_estimin_m[2], real_T rtu_states_estimin_l, const real_T
  rtu_states_estimin_m4[3], real_T rtu_states_estimin_ly, const real_T
  rtu_states_estimin_le[3], B_ControllerPolePlace_Drone_C_T *localB,
  P_ControllerPolePlace_Drone_C_T *localP, P_Drone_Compensator_T
  *Drone_Compensator_P);
extern void Drone_Compens_MeasurementUpdate(boolean_T rtu_Enable, const real_T
  rtu_Lk[4], const real_T rtu_yk[2], const real_T rtu_yhatkk1[2],
  B_MeasurementUpdate_Drone_Com_T *localB);
extern void Drone_Compe_UseCurrentEstimator(boolean_T rtu_Enablek, const real_T
  rtu_Mk[4], const real_T rtu_uk[2], const real_T rtu_yk[2], const real_T
  rtu_Ck[4], const real_T rtu_Dk[4], const real_T rtu_xhatkk1[2],
  B_UseCurrentEstimator_Drone_C_T *localB);
extern void Drone_Co_Drone_Compensator_Init(DW_Drone_Compensator_Drone_Co_T
  *localDW, P_Drone_Compensator_Drone_Com_T *localP);
extern void Drone_Compens_Drone_Compensator(boolean_T
  rtu_controlModePosVSOrient_flag, const real_T rtu_pos_refin[3], boolean_T
  rtu_takeoff_flag, const real_T rtu_orient_refin[3], real_T
  rtu_sensordata_datin, real_T rtu_sensordata_datin_n, real_T
  rtu_sensordata_datin_j, real_T rtu_sensordata_datin_k, real_T
  rtu_sensordata_datin_c, real_T rtu_sensordata_datin_g, real_T
  rtu_sensordata_datin_gz, real_T rtu_sensordata_datin_p, const real_T
  rtu_opticalFlow_datin[3], const real_T rtu_sensordataCalib_datin[7], const
  real_T rtu_posVIS_datin[4], real_T rtu_usePosVIS_flagin, const real_T
  rtu_batteryStatus_datin[2], B_Drone_Compensator_Drone_Com_T *localB,
  DW_Drone_Compensator_Drone_Co_T *localDW, P_Drone_Compensator_Drone_Com_T
  *localP, P_Drone_Compensator_T *Drone_Compensator_P);

#endif                                 /* RTW_HEADER_Drone_Compensator_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
