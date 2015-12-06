/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Drone_Compensator.c
 *
 * Code generated for Simulink model 'Drone_Compensator'.
 *
 * Model version                  : 1.3012
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Dec  5 23:16:06 2015
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

#include "Drone_Compensator.h"
#include "Drone_Compensator_private.h"

/* Initial conditions for atomic system: '<S1>/ControllerPID' */
void Drone_Compen_ControllerPID_Init(DW_ControllerPID_Drone_Compen_T *localDW,
  P_ControllerPID_Drone_Compens_T *localP)
{
  /* InitializeConditions for Delay: '<S2>/Delay' */
  localDW->Delay_DSTATE[0] = localP->Delay_InitialCondition;
  localDW->Delay_DSTATE[1] = localP->Delay_InitialCondition;

  /* InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' */
  localDW->DiscreteTimeIntegrator_DSTATE[0] = localP->DiscreteTimeIntegrator_IC;
  localDW->DiscreteTimeIntegrator_DSTATE[1] = localP->DiscreteTimeIntegrator_IC;
}

/* Output and update for atomic system: '<S1>/ControllerPID' */
void Drone_Compensator_ControllerPID(const real_T rtu_pos_ref[3], boolean_T
  rtu_takeoff_flag, const real_T rtu_orient_ref[3], boolean_T
  rtu_controlModePosVOrient_flagi, const real_T rtu_states_estim[3], const
  real_T rtu_states_estim_m[3], real_T rtu_states_estim_l, real_T
  rtu_states_estim_b, const real_T rtu_states_estim_o[2], const real_T
  rtu_states_estim_p[2], B_ControllerPID_Drone_Compens_T *localB,
  DW_ControllerPID_Drone_Compen_T *localDW, P_ControllerPID_Drone_Compens_T
  *localP, P_Drone_Compensator_T *Drone_Compensator_P)
{
  /* local block i/o variables */
  real_T rtb_SaturationThrust;
  real_T tmp[4];
  int32_T i;
  real_T rtb_pitchrollerror_idx_0;
  real_T rtb_pitchrollerror_idx_1;
  real_T rtb_antiWU_Gain_idx_0;
  real_T rtb_antiWU_Gain_idx_1;
  real_T tmp_0;
  real_T tmp_1;
  real_T u2;
  real_T tmp_2;

  /* Inport: '<S2>/pos_ref' */
  localB->pos_ref[0] = rtu_pos_ref[0];
  localB->pos_ref[1] = rtu_pos_ref[1];
  localB->pos_ref[2] = rtu_pos_ref[2];

  /* Switch: '<S2>/TakeoffOrControl_Switch' incorporates:
   *  Constant: '<S2>/w0'
   *  Gain: '<S2>/D_z'
   *  Gain: '<S2>/P_z'
   *  Gain: '<S2>/takeoff_gain'
   *  Sum: '<S2>/Sum15'
   *  Sum: '<S2>/Sum3'
   */
  if (rtu_takeoff_flag) {
    rtb_SaturationThrust = -Drone_Compensator_P->quad.g *
      Drone_Compensator_P->quad.M *
      Drone_Compensator_P->controlHelperParams.takeoff_gain;
  } else {
    rtb_SaturationThrust = (localB->pos_ref[2] - rtu_states_estim_b) *
      localP->P_z_Gain - localP->D_z_Gain * rtu_states_estim_l;
  }

  /* End of Switch: '<S2>/TakeoffOrControl_Switch' */

  /* Sum: '<S2>/Sum4' incorporates:
   *  Constant: '<S2>/w0'
   */
  rtb_SaturationThrust += -Drone_Compensator_P->quad.g *
    Drone_Compensator_P->quad.M;

  /* Inport: '<S2>/orient_ref' */
  /* MATLAB Function 'Drone_Compensator/ControllerPID/inverse rotation Function': '<S5>:1' */
  /* inverse yaw-rotation */
  /* '<S5>:1:3' */
  localB->orient_ref[0] = rtu_orient_ref[0];
  localB->orient_ref[1] = rtu_orient_ref[1];
  localB->orient_ref[2] = rtu_orient_ref[2];

  /* Switch: '<S2>/Switch_refAtt' incorporates:
   *  Gain: '<S2>/D_xy'
   *  Gain: '<S2>/P_xy'
   *  MATLAB Function: '<S2>/inverse rotation Function'
   *  Product: '<S2>/Product'
   *  Sum: '<S2>/Sum18'
   */
  if (rtu_controlModePosVOrient_flagi) {
    /* Sum: '<S2>/Sum17' incorporates:
     *  Product: '<S2>/Product'
     */
    tmp_0 = localB->pos_ref[0] - rtu_states_estim_p[0];
    tmp_1 = localB->pos_ref[1] - rtu_states_estim_p[1];
    localB->Switch_refAtt[0] = (cos(rtu_states_estim[0]) * tmp_0 + sin
      (rtu_states_estim[0]) * tmp_1) * localP->P_xy_Gain[0] + localP->D_xy_Gain
      [0] * rtu_states_estim_o[0];
    localB->Switch_refAtt[1] = (-sin(rtu_states_estim[0]) * tmp_0 + cos
      (rtu_states_estim[0]) * tmp_1) * localP->P_xy_Gain[1] + localP->D_xy_Gain
      [1] * rtu_states_estim_o[1];
  } else {
    localB->Switch_refAtt[0] = localB->orient_ref[1];
    localB->Switch_refAtt[1] = localB->orient_ref[2];
  }

  /* End of Switch: '<S2>/Switch_refAtt' */

  /* Sum: '<S2>/Sum19' */
  rtb_pitchrollerror_idx_0 = localB->Switch_refAtt[0] - rtu_states_estim[1];
  rtb_pitchrollerror_idx_1 = localB->Switch_refAtt[1] - rtu_states_estim[2];

  /* Gain: '<S2>/antiWU_Gain' incorporates:
   *  Delay: '<S2>/Delay'
   */
  rtb_antiWU_Gain_idx_0 = localP->antiWU_Gain_Gain * localDW->Delay_DSTATE[0];
  rtb_antiWU_Gain_idx_1 = localP->antiWU_Gain_Gain * localDW->Delay_DSTATE[1];

  /* Saturate: '<S2>/SaturationThrust' */
  tmp_0 = -4.0 *
    Drone_Compensator_P->controlHelperParams.totalThrust_maxRelative *
    Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;
  u2 = 4.0 * Drone_Compensator_P->controlHelperParams.totalThrust_maxRelative *
    Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;
  if (rtb_SaturationThrust > u2) {
    rtb_SaturationThrust = u2;
  } else {
    if (rtb_SaturationThrust < tmp_0) {
      rtb_SaturationThrust = tmp_0;
    }
  }

  /* End of Saturate: '<S2>/SaturationThrust' */

  /* SignalConversion: '<S4>/TmpSignal ConversionAtProductInport2' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
   *  Gain: '<S2>/D_pr'
   *  Gain: '<S2>/D_yaw'
   *  Gain: '<S2>/I_pr'
   *  Gain: '<S2>/P_pr'
   *  Gain: '<S2>/P_yaw'
   *  Sum: '<S2>/Sum1'
   *  Sum: '<S2>/Sum16'
   *  Sum: '<S2>/Sum2'
   */
  tmp_1 = (localB->orient_ref[0] - rtu_states_estim[0]) * localP->P_yaw_Gain -
    localP->D_yaw_Gain * rtu_states_estim_m[2];
  tmp_0 = (localP->P_pr_Gain[0] * rtb_pitchrollerror_idx_0 + localP->I_pr_Gain *
           localDW->DiscreteTimeIntegrator_DSTATE[0]) - localP->D_pr_Gain[0] *
    rtu_states_estim_m[1];
  u2 = (localP->P_pr_Gain[1] * rtb_pitchrollerror_idx_1 + localP->I_pr_Gain *
        localDW->DiscreteTimeIntegrator_DSTATE[1]) - localP->D_pr_Gain[1] *
    rtu_states_estim_m[0];

  /* Product: '<S4>/Product' incorporates:
   *  Constant: '<S4>/TorquetotalThrustToThrustperMotor'
   *  Gain: '<S6>/thrustToMotorcommand'
   *  SignalConversion: '<S4>/TmpSignal ConversionAtProductInport2'
   */
  for (i = 0; i < 4; i++) {
    tmp_2 = Drone_Compensator_P->controlHelperParams.Q2Ts[i + 12] * u2 +
      (Drone_Compensator_P->controlHelperParams.Q2Ts[i + 8] * tmp_0 +
       (Drone_Compensator_P->controlHelperParams.Q2Ts[i + 4] * tmp_1 +
        Drone_Compensator_P->controlHelperParams.Q2Ts[i] * rtb_SaturationThrust));
    tmp[i] = tmp_2;
  }

  /* End of Product: '<S4>/Product' */

  /* Gain: '<S6>/Motordirections1' incorporates:
   *  Gain: '<S6>/thrustToMotorcommand'
   *  Saturate: '<S6>/Saturation5'
   */
  tmp_0 = -Drone_Compensator_P->quadEDT.thrustToMotorcommand * tmp[0];
  if (tmp_0 > Drone_Compensator_P->quadEDT.motors_max) {
    tmp_0 = Drone_Compensator_P->quadEDT.motors_max;
  } else {
    if (tmp_0 < localP->Saturation5_LowerSat) {
      tmp_0 = localP->Saturation5_LowerSat;
    }
  }

  localB->Motordirections1[0] = localP->Motordirections1_Gain[0] * tmp_0;
  tmp_0 = -Drone_Compensator_P->quadEDT.thrustToMotorcommand * tmp[1];
  if (tmp_0 > Drone_Compensator_P->quadEDT.motors_max) {
    tmp_0 = Drone_Compensator_P->quadEDT.motors_max;
  } else {
    if (tmp_0 < localP->Saturation5_LowerSat) {
      tmp_0 = localP->Saturation5_LowerSat;
    }
  }

  localB->Motordirections1[1] = localP->Motordirections1_Gain[1] * tmp_0;
  tmp_0 = -Drone_Compensator_P->quadEDT.thrustToMotorcommand * tmp[2];
  if (tmp_0 > Drone_Compensator_P->quadEDT.motors_max) {
    tmp_0 = Drone_Compensator_P->quadEDT.motors_max;
  } else {
    if (tmp_0 < localP->Saturation5_LowerSat) {
      tmp_0 = localP->Saturation5_LowerSat;
    }
  }

  localB->Motordirections1[2] = localP->Motordirections1_Gain[2] * tmp_0;
  tmp_0 = -Drone_Compensator_P->quadEDT.thrustToMotorcommand * tmp[3];
  if (tmp_0 > Drone_Compensator_P->quadEDT.motors_max) {
    tmp_0 = Drone_Compensator_P->quadEDT.motors_max;
  } else {
    if (tmp_0 < localP->Saturation5_LowerSat) {
      tmp_0 = localP->Saturation5_LowerSat;
    }
  }

  localB->Motordirections1[3] = localP->Motordirections1_Gain[3] * tmp_0;

  /* End of Gain: '<S6>/Motordirections1' */

  /* Update for Delay: '<S2>/Delay' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
   */
  localDW->Delay_DSTATE[0] = localDW->DiscreteTimeIntegrator_DSTATE[0];
  localDW->Delay_DSTATE[1] = localDW->DiscreteTimeIntegrator_DSTATE[1];

  /* Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
   *  Sum: '<S2>/Add'
   */
  localDW->DiscreteTimeIntegrator_DSTATE[0] += (rtb_pitchrollerror_idx_0 -
    rtb_antiWU_Gain_idx_0) * localP->DiscreteTimeIntegrator_gainval;
  localDW->DiscreteTimeIntegrator_DSTATE[1] += (rtb_pitchrollerror_idx_1 -
    rtb_antiWU_Gain_idx_1) * localP->DiscreteTimeIntegrator_gainval;
  if (localDW->DiscreteTimeIntegrator_DSTATE[0] >=
      localP->DiscreteTimeIntegrator_UpperSat) {
    localDW->DiscreteTimeIntegrator_DSTATE[0] =
      localP->DiscreteTimeIntegrator_UpperSat;
  } else {
    if (localDW->DiscreteTimeIntegrator_DSTATE[0] <=
        localP->DiscreteTimeIntegrator_LowerSat) {
      localDW->DiscreteTimeIntegrator_DSTATE[0] =
        localP->DiscreteTimeIntegrator_LowerSat;
    }
  }

  if (localDW->DiscreteTimeIntegrator_DSTATE[1] >=
      localP->DiscreteTimeIntegrator_UpperSat) {
    localDW->DiscreteTimeIntegrator_DSTATE[1] =
      localP->DiscreteTimeIntegrator_UpperSat;
  } else {
    if (localDW->DiscreteTimeIntegrator_DSTATE[1] <=
        localP->DiscreteTimeIntegrator_LowerSat) {
      localDW->DiscreteTimeIntegrator_DSTATE[1] =
        localP->DiscreteTimeIntegrator_LowerSat;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' */
}

/*
 * Output and update for enable system:
 *    '<S92>/MeasurementUpdate'
 *    '<S152>/MeasurementUpdate'
 */
void Drone_Compens_MeasurementUpdate(boolean_T rtu_Enable, const real_T rtu_Lk[4],
  const real_T rtu_yk[2], const real_T rtu_yhatkk1[2],
  B_MeasurementUpdate_Drone_Com_T *localB)
{
  real_T rtu_yk_idx_0;
  real_T rtu_yk_idx_1;

  /* Outputs for Enabled SubSystem: '<S92>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S117>/Enable'
   */
  if (rtu_Enable) {
    /* Sum: '<S117>/Sum' incorporates:
     *  Product: '<S117>/Product3'
     */
    rtu_yk_idx_0 = rtu_yk[0] - rtu_yhatkk1[0];
    rtu_yk_idx_1 = rtu_yk[1] - rtu_yhatkk1[1];

    /* Product: '<S117>/Product3' */
    localB->Product3[0] = 0.0;
    localB->Product3[0] += rtu_Lk[0] * rtu_yk_idx_0;
    localB->Product3[0] += rtu_Lk[2] * rtu_yk_idx_1;
    localB->Product3[1] = 0.0;
    localB->Product3[1] += rtu_Lk[1] * rtu_yk_idx_0;
    localB->Product3[1] += rtu_Lk[3] * rtu_yk_idx_1;
  }

  /* End of Outputs for SubSystem: '<S92>/MeasurementUpdate' */
}

/*
 * Output and update for atomic system:
 *    '<S68>/UseCurrentEstimator'
 *    '<S131>/UseCurrentEstimator'
 */
void Drone_Compe_UseCurrentEstimator(boolean_T rtu_Enablek, const real_T rtu_Mk
  [4], const real_T rtu_uk[2], const real_T rtu_yk[2], const real_T rtu_Ck[4],
  const real_T rtu_Dk[4], const real_T rtu_xhatkk1[2],
  B_UseCurrentEstimator_Drone_C_T *localB)
{
  real_T rtu_yk_idx_0;
  real_T rtu_yk_idx_1;

  /* Outputs for Enabled SubSystem: '<S97>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S118>/Enable'
   */
  if (rtu_Enablek) {
    /* Sum: '<S118>/Add1' incorporates:
     *  Product: '<S118>/Product'
     *  Product: '<S118>/Product1'
     */
    rtu_yk_idx_0 = (rtu_yk[0] - (rtu_Ck[0] * rtu_xhatkk1[0] + rtu_Ck[2] *
      rtu_xhatkk1[1])) - (rtu_Dk[0] * rtu_uk[0] + rtu_Dk[2] * rtu_uk[1]);
    rtu_yk_idx_1 = (rtu_yk[1] - (rtu_Ck[1] * rtu_xhatkk1[0] + rtu_Ck[3] *
      rtu_xhatkk1[1])) - (rtu_Dk[1] * rtu_uk[0] + rtu_Dk[3] * rtu_uk[1]);

    /* Product: '<S118>/Product2' */
    localB->Product2[0] = 0.0;
    localB->Product2[0] += rtu_Mk[0] * rtu_yk_idx_0;
    localB->Product2[0] += rtu_Mk[2] * rtu_yk_idx_1;
    localB->Product2[1] = 0.0;
    localB->Product2[1] += rtu_Mk[1] * rtu_yk_idx_0;
    localB->Product2[1] += rtu_Mk[3] * rtu_yk_idx_1;
  }

  /* End of Outputs for SubSystem: '<S97>/Enabled Subsystem' */

  /* Sum: '<S97>/Add' */
  localB->Add[0] = localB->Product2[0] + rtu_xhatkk1[0];
  localB->Add[1] = localB->Product2[1] + rtu_xhatkk1[1];
}

/* Initial conditions for atomic system: '<Root>/Drone_Compensator' */
void Drone_Co_Drone_Compensator_Init(DW_Drone_Compensator_Drone_Co_T *localDW,
  P_Drone_Compensator_Drone_Com_T *localP)
{
  int32_T i;

  /* InitializeConditions for DiscreteFir: '<S10>/FIR_IMUaccel' */
  localDW->FIR_IMUaccel_circBuf = 0;
  for (i = 0; i < 15; i++) {
    localDW->FIR_IMUaccel_states[i] = localP->FIR_IMUaccel_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFir: '<S10>/FIR_IMUaccel' */

  /* InitializeConditions for DiscreteFilter: '<S10>/IIR_IMUgyro_r' */
  for (i = 0; i < 5; i++) {
    localDW->IIR_IMUgyro_r_states[i] = localP->IIR_IMUgyro_r_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFilter: '<S10>/IIR_IMUgyro_r' */

  /* InitializeConditions for MATLAB Function: '<S3>/EstimatorOrientation' */
  localDW->yaw_cur = 0.0;
  localDW->pitch_cur = 0.0;
  localDW->roll_cur = 0.0;

  /* InitializeConditions for Delay: '<S66>/Delay' */
  localDW->Delay_DSTATE[0] = localP->Delay_InitialCondition;
  localDW->Delay_DSTATE[1] = localP->Delay_InitialCondition;

  /* InitializeConditions for DiscreteFilter: '<S69>/IIRgyroz' */
  for (i = 0; i < 10; i++) {
    localDW->IIRgyroz_states[i] = localP->IIRgyroz_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFilter: '<S69>/IIRgyroz' */

  /* InitializeConditions for UnitDelay: '<S119>/UD' */
  localDW->UD_DSTATE[0] = localP->DiscreteDerivative_ICPrevScaled;
  localDW->UD_DSTATE[1] = localP->DiscreteDerivative_ICPrevScaled;

  /* InitializeConditions for Delay: '<S65>/Delay' */
  localDW->Delay_DSTATE_b[0] = localP->Delay_InitialCondition_h;
  localDW->Delay_DSTATE_b[1] = localP->Delay_InitialCondition_h;

  /* InitializeConditions for Delay: '<S7>/Delay2' */
  localDW->Delay2_DSTATE = localP->Delay2_InitialCondition;
  for (i = 0; i < 5; i++) {
    /* InitializeConditions for DiscreteFilter: '<S12>/pressureFilter_IIR' */
    localDW->pressureFilter_IIR_states[i] =
      localP->pressureFilter_IIR_InitialState;

    /* InitializeConditions for DiscreteFilter: '<S12>/soonarFilter_IIR' */
    localDW->soonarFilter_IIR_states[i] = localP->soonarFilter_IIR_InitialStates;
  }

  /* InitializeConditions for Delay: '<S11>/MemoryX' */
  localDW->icLoad = 1U;

  /* InitializeConditions for Delay: '<S68>/MemoryX' */
  localDW->icLoad_l = 1U;

  /* InitializeConditions for Delay: '<S3>/Delay1' */
  localDW->Delay1_DSTATE[0] = localP->Delay1_InitialCondition;
  localDW->Delay1_DSTATE[1] = localP->Delay1_InitialCondition;

  /* InitializeConditions for Delay: '<S131>/MemoryX' */
  localDW->icLoad_j = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S66>/SimplyIntegrateVelocity' */
  localDW->SimplyIntegrateVelocity_DSTATE[0] =
    localP->SimplyIntegrateVelocity_IC;
  localDW->SimplyIntegrateVelocity_DSTATE[1] =
    localP->SimplyIntegrateVelocity_IC;
  localDW->SimplyIntegrateVelocity_PrevRes = 2;

  /* InitializeConditions for Atomic SubSystem: '<S1>/ControllerPID' */
  Drone_Compen_ControllerPID_Init(&localDW->ControllerPID,
    (P_ControllerPID_Drone_Compens_T *)&localP->ControllerPID);

  /* End of InitializeConditions for SubSystem: '<S1>/ControllerPID' */
}

/* Output and update for atomic system: '<Root>/Drone_Compensator' */
void Drone_Compens_Drone_Compensator(boolean_T rtu_controlModePosVSOrient_flag,
  const real_T rtu_pos_refin[3], boolean_T rtu_takeoff_flag, const real_T
  rtu_orient_refin[3], real_T rtu_sensordata_datin, real_T
  rtu_sensordata_datin_n, real_T rtu_sensordata_datin_j, real_T
  rtu_sensordata_datin_k, real_T rtu_sensordata_datin_c, real_T
  rtu_sensordata_datin_g, real_T rtu_sensordata_datin_gz, real_T
  rtu_sensordata_datin_p, const real_T rtu_opticalFlow_datin[3], const real_T
  rtu_sensordataCalib_datin[7], const real_T rtu_posVIS_datin[4], real_T
  rtu_usePosVIS_flagin, const real_T rtu_batteryStatus_datin[2],
  B_Drone_Compensator_Drone_Com_T *localB, DW_Drone_Compensator_Drone_Co_T
  *localDW, P_Drone_Compensator_Drone_Com_T *localP, P_Drone_Compensator_T
  *Drone_Compensator_P)
{
  /* local block i/o variables */
  real_T rtb_opticalFlowToVelocity_gain[3];
  real_T rtb_invertzaxisGain;
  real_T rtb_altfrompress;
  real_T rtb_deuler_datout[3];
  real_T rtb_Ckxhatkk1_e;
  real_T rtb_Dk1uk1_p;
  boolean_T rtb_LogicalOperator3_j;
  int32_T k;
  int32_T j;
  int32_T memOffset;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T rtb_Sum[3];
  real_T rtb_Add1_l;
  boolean_T rtb_LogicalOperator3;
  real_T rtb_vel_world[3];
  real_T rtb_r;
  real_T rtb_Dk1uk1[2];
  real_T rtb_Add1[2];
  boolean_T rtb_nicemeasurementornewupdaten;
  boolean_T rtb_LogicalOperator_o;
  boolean_T rtb_Compare_l;
  boolean_T rtb_Compare_cz;
  boolean_T rtb_Compare_po;
  boolean_T rtb_Compare_du;
  boolean_T rtb_Compare_ii;
  real_T inverseIMU_gain[6];
  real_T soonarFilter_IIR_tmp;
  real_T pressureFilter_IIR_tmp;
  real_T IIR_IMUgyro_r_tmp;
  int32_T i;
  real_T tmp[9];
  real_T tmp_0[9];
  real_T tmp_1[9];
  real_T tmp_2[9];
  real_T tmp_3[9];
  real_T tmp_4[9];
  real_T rtb_TmpSignalConversionAtSFun_0;

  /* Inport: '<S1>/takeoff_flag' */
  localB->takeoff_flag = rtu_takeoff_flag;

  /* Inport: '<S1>/controlModePosVSOrient_flagin' */
  localB->controlModePosVSOrient_flagin = rtu_controlModePosVSOrient_flag;

  /* Inport: '<S1>/posVIS_datin' */
  localB->posVIS_datin[0] = rtu_posVIS_datin[0];
  localB->posVIS_datin[1] = rtu_posVIS_datin[1];
  localB->posVIS_datin[2] = rtu_posVIS_datin[2];
  localB->posVIS_datin[3] = rtu_posVIS_datin[3];

  /* Inport: '<S1>/sensordataCalib_datin' */
  for (i = 0; i < 7; i++) {
    localB->sensordataCalib_datin[i] = rtu_sensordataCalib_datin[i];
  }

  /* End of Inport: '<S1>/sensordataCalib_datin' */

  /* Inport: '<S1>/sensordata_datin' */
  localB->sensordata_datin[0] = rtu_sensordata_datin;
  localB->sensordata_datin[1] = rtu_sensordata_datin_n;
  localB->sensordata_datin[2] = rtu_sensordata_datin_j;
  localB->sensordata_datin[3] = rtu_sensordata_datin_k;
  localB->sensordata_datin[4] = rtu_sensordata_datin_c;
  localB->sensordata_datin[5] = rtu_sensordata_datin_g;
  localB->sensordata_datin[6] = rtu_sensordata_datin_gz;
  localB->sensordata_datin[7] = rtu_sensordata_datin_p;

  /* Inport: '<S1>/usePosVIS_flagin' */
  localB->usePosVIS_flagin = rtu_usePosVIS_flagin;

  /* Inport: '<S1>/opticalFlow_datin' */
  localB->opticalFlow_datin[0] = rtu_opticalFlow_datin[0];
  localB->opticalFlow_datin[1] = rtu_opticalFlow_datin[1];
  localB->opticalFlow_datin[2] = rtu_opticalFlow_datin[2];

  /* Gain: '<S10>/inverseIMU_gain' incorporates:
   *  Bias: '<S10>/Assuming that calib was done level!'
   *  Sum: '<S10>/Sum1'
   */
  for (i = 0; i < 6; i++) {
    inverseIMU_gain[i] = (localB->sensordata_datin[i] -
                          (localB->sensordataCalib_datin[i] +
      localP->Assumingthatcalibwasdonelevel_B[i])) *
      Drone_Compensator_P->quadEDT.inverseIMU_gain[i];
  }

  /* End of Gain: '<S10>/inverseIMU_gain' */

  /* DiscreteFir: '<S10>/FIR_IMUaccel' */
  rtb_Add1_l = inverseIMU_gain[0] * localP->FIR_IMUaccel_Coefficients[0];
  i = 1;
  for (j = localDW->FIR_IMUaccel_circBuf; j < 5; j++) {
    rtb_Add1_l += localDW->FIR_IMUaccel_states[j] *
      localP->FIR_IMUaccel_Coefficients[i];
    i++;
  }

  for (j = 0; j < localDW->FIR_IMUaccel_circBuf; j++) {
    rtb_Add1_l += localDW->FIR_IMUaccel_states[j] *
      localP->FIR_IMUaccel_Coefficients[i];
    i++;
  }

  localB->FIR_IMUaccel[0] = rtb_Add1_l;
  rtb_Add1_l = inverseIMU_gain[1] * localP->FIR_IMUaccel_Coefficients[0];
  i = 1;
  for (j = localDW->FIR_IMUaccel_circBuf; j < 5; j++) {
    rtb_Add1_l += localDW->FIR_IMUaccel_states[5 + j] *
      localP->FIR_IMUaccel_Coefficients[i];
    i++;
  }

  for (j = 0; j < localDW->FIR_IMUaccel_circBuf; j++) {
    rtb_Add1_l += localDW->FIR_IMUaccel_states[5 + j] *
      localP->FIR_IMUaccel_Coefficients[i];
    i++;
  }

  localB->FIR_IMUaccel[1] = rtb_Add1_l;
  rtb_Add1_l = inverseIMU_gain[2] * localP->FIR_IMUaccel_Coefficients[0];
  i = 1;
  for (j = localDW->FIR_IMUaccel_circBuf; j < 5; j++) {
    rtb_Add1_l += localDW->FIR_IMUaccel_states[10 + j] *
      localP->FIR_IMUaccel_Coefficients[i];
    i++;
  }

  for (j = 0; j < localDW->FIR_IMUaccel_circBuf; j++) {
    rtb_Add1_l += localDW->FIR_IMUaccel_states[10 + j] *
      localP->FIR_IMUaccel_Coefficients[i];
    i++;
  }

  localB->FIR_IMUaccel[2] = rtb_Add1_l;

  /* End of DiscreteFir: '<S10>/FIR_IMUaccel' */

  /* DiscreteFilter: '<S10>/IIR_IMUgyro_r' */
  IIR_IMUgyro_r_tmp = inverseIMU_gain[5];
  i = 1;
  for (j = 0; j < 5; j++) {
    IIR_IMUgyro_r_tmp -= localP->IIR_IMUgyro_r_DenCoef[i] *
      localDW->IIR_IMUgyro_r_states[j];
    i++;
  }

  IIR_IMUgyro_r_tmp /= localP->IIR_IMUgyro_r_DenCoef[0];
  rtb_r = localP->IIR_IMUgyro_r_NumCoef[0] * IIR_IMUgyro_r_tmp;
  i = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += localP->IIR_IMUgyro_r_NumCoef[i] * localDW->IIR_IMUgyro_r_states[j];
    i++;
  }

  /* End of DiscreteFilter: '<S10>/IIR_IMUgyro_r' */

  /* SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  MATLAB Function: '<S3>/EstimatorOrientation'
   */
  rtb_TmpSignalConversionAtSFun_0 = rtb_r;

  /* MATLAB Function: '<S3>/EstimatorOrientation' incorporates:
   *  Constant: '<S184>/Constant'
   *  Constant: '<S3>/sampleTime'
   *  Constant: '<S3>/sampleTime1'
   *  Logic: '<S10>/Logical Operator'
   *  RelationalOperator: '<S184>/Compare'
   *  SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorOrientation': '<S8>:1' */
  /* '<S8>:1:4' */
  /* '<S8>:1:5' */
  /* '<S8>:1:6' */
  /* % Estimating attitude */
  /*  =============================== */
  /*  PURPOSE estimates the orientation fusing gyroscopic and accelerometer measurements with potentially available vision updates on yaw */
  /*  INSPIRED by http://www.pieter-jan.com/node/11  */
  /*  ADAPTED by Fabian Riether */
  /*  UPDATE DATE 2015/08/25 */
  /*  SPECIAL NOTES */
  /*  =============================== */
  /*   2015/08/25 created */
  /*  ================================== */
  /* PARAMS */
  /* -- */
  /* Rotation of angular velocity vector from Bodyframe to Worldframe, inverted Wronskian (body rates p-q-r to euler rates yaw pitch roll) */
  rtb_Add1_l = cos(localDW->pitch_cur);
  tmp[0] = 0.0;
  tmp[3] = sin(localDW->roll_cur);
  tmp[6] = cos(localDW->roll_cur);
  tmp[1] = 0.0;
  tmp[4] = cos(localDW->roll_cur) * cos(localDW->pitch_cur);
  tmp[7] = -sin(localDW->roll_cur) * cos(localDW->pitch_cur);
  tmp[2] = cos(localDW->pitch_cur);
  tmp[5] = sin(localDW->roll_cur) * sin(localDW->pitch_cur);
  tmp[8] = cos(localDW->roll_cur) * sin(localDW->pitch_cur);
  for (i = 0; i < 3; i++) {
    tmp_0[3 * i] = tmp[3 * i] / rtb_Add1_l;
    tmp_0[1 + 3 * i] = tmp[3 * i + 1] / rtb_Add1_l;
    tmp_0[2 + 3 * i] = tmp[3 * i + 2] / rtb_Add1_l;
  }

  for (i = 0; i < 3; i++) {
    rtb_deuler_datout[i] = 0.0;
    rtb_deuler_datout[i] += tmp_0[i] * inverseIMU_gain[3];
    rtb_deuler_datout[i] += tmp_0[i + 3] * inverseIMU_gain[4];
    rtb_deuler_datout[i] += tmp_0[i + 6] * rtb_r;
  }

  /* Integrate gyroscope data */
  rtb_r = rtb_deuler_datout[2] * Drone_Compensator_P->quadEDT.sampletime +
    localDW->roll_cur;
  rtb_Add1_l = rtb_deuler_datout[1] * Drone_Compensator_P->quadEDT.sampletime +
    localDW->pitch_cur;
  pressureFilter_IIR_tmp = rtb_deuler_datout[0] *
    Drone_Compensator_P->quadEDT.sampletime + localDW->yaw_cur;

  /* Compensate for drift with accelerometer data if in un-accelerated flight */
  scale = 2.2250738585072014E-308;
  absxk = fabs(localB->FIR_IMUaccel[0]);
  if (absxk > 2.2250738585072014E-308) {
    soonarFilter_IIR_tmp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    soonarFilter_IIR_tmp = t * t;
  }

  absxk = fabs(localB->FIR_IMUaccel[1]);
  if (absxk > scale) {
    t = scale / absxk;
    soonarFilter_IIR_tmp = soonarFilter_IIR_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    soonarFilter_IIR_tmp += t * t;
  }

  absxk = fabs(localB->FIR_IMUaccel[2]);
  if (absxk > scale) {
    t = scale / absxk;
    soonarFilter_IIR_tmp = soonarFilter_IIR_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    soonarFilter_IIR_tmp += t * t;
  }

  soonarFilter_IIR_tmp = scale * sqrt(soonarFilter_IIR_tmp);
  if ((soonarFilter_IIR_tmp > 0.998 * Drone_Compensator_P->quad.g) &&
      (soonarFilter_IIR_tmp < 1.002 * Drone_Compensator_P->quad.g)) {
    rtb_r = atan(localB->FIR_IMUaccel[1] / localB->FIR_IMUaccel[2]) * 0.001 +
      rtb_r * 0.999;
    rtb_Add1_l = asin(localB->FIR_IMUaccel[0] / Drone_Compensator_P->quad.g) *
      0.001 + rtb_Add1_l * 0.999;
  }

  /* Compensate yaw-bias/drift to world yaw if measurement form visually reconstructed pose based on marker setup available */
  if ((localB->posVIS_datin[0] != Drone_Compensator_P->quadEDT.NO_VIS_X) &&
      (localB->usePosVIS_flagin != 0.0)) {
    pressureFilter_IIR_tmp = 0.8 * pressureFilter_IIR_tmp + 0.2 *
      localB->posVIS_datin[3];
  }

  /* '<S8>:1:18' */
  /* '<S8>:1:20' */
  localDW->yaw_cur = pressureFilter_IIR_tmp;

  /* '<S8>:1:21' */
  localDW->pitch_cur = rtb_Add1_l;

  /* '<S8>:1:22' */
  localDW->roll_cur = rtb_r;

  /* '<S8>:1:24' */
  localB->orient_estimout[0] = pressureFilter_IIR_tmp;
  localB->orient_estimout[1] = rtb_Add1_l;
  localB->orient_estimout[2] = rtb_r;

  /* '<S8>:1:25' */
  localB->dorient_estimout[0] = inverseIMU_gain[3];
  localB->dorient_estimout[1] = inverseIMU_gain[4];
  localB->dorient_estimout[2] = rtb_TmpSignalConversionAtSFun_0;

  /* Abs: '<S132>/Abs' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[1]);

  /* RelationalOperator: '<S181>/Compare' incorporates:
   *  Constant: '<S181>/Constant'
   */
  rtb_Compare_ii = (rtb_Dk1uk1_p <= localP->maxp3_const);

  /* Abs: '<S132>/Abs1' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[2]);

  /* MATLAB Function: '<S132>/abs' incorporates:
   *  Delay: '<S66>/Delay'
   *  Sum: '<S132>/Add1'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs': '<S179>:1' */
  /* '<S179>:1:2' */
  scale = 2.2250738585072014E-308;
  absxk = fabs(localB->posVIS_datin[0] - localDW->Delay_DSTATE[0]);
  if (absxk > 2.2250738585072014E-308) {
    rtb_r = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    rtb_r = t * t;
  }

  absxk = fabs(localB->posVIS_datin[1] - localDW->Delay_DSTATE[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_r = rtb_r * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_r += t * t;
  }

  rtb_r = scale * sqrt(rtb_r);

  /* End of MATLAB Function: '<S132>/abs' */

  /* Logic: '<S132>/Logical Operator3' incorporates:
   *  Constant: '<S180>/Constant'
   *  Constant: '<S182>/Constant'
   *  Constant: '<S183>/Constant'
   *  RelationalOperator: '<S180>/Compare'
   *  RelationalOperator: '<S182>/Compare'
   *  RelationalOperator: '<S183>/Compare'
   */
  rtb_LogicalOperator3 = ((localB->posVIS_datin[0] !=
    localP->checkifPosavailable_const) && rtb_Compare_ii && (rtb_Dk1uk1_p <=
    localP->maxq3_const) && (rtb_r < localP->planarjumpsVISPOS_const));

  /* Abs: '<S69>/Abs2' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[1]);

  /* RelationalOperator: '<S122>/Compare' incorporates:
   *  Constant: '<S122>/Constant'
   */
  rtb_Compare_ii = (rtb_Dk1uk1_p <= localP->maxp_const);

  /* Abs: '<S69>/Abs3' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[2]);

  /* RelationalOperator: '<S124>/Compare' incorporates:
   *  Constant: '<S124>/Constant'
   */
  rtb_Compare_du = (rtb_Dk1uk1_p <= localP->maxq_const);

  /* Abs: '<S69>/Abs' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[3]);

  /* RelationalOperator: '<S126>/Compare' incorporates:
   *  Constant: '<S126>/Constant'
   */
  rtb_Compare_po = (rtb_Dk1uk1_p <= localP->maxw1_const);

  /* Abs: '<S69>/Abs1' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[4]);

  /* RelationalOperator: '<S127>/Compare' incorporates:
   *  Constant: '<S127>/Constant'
   */
  rtb_Compare_cz = (rtb_Dk1uk1_p <= localP->maxw2_const);

  /* DiscreteFilter: '<S69>/IIRgyroz' */
  for (k = 0; k < 2; k++) {
    memOffset = k * 5;
    rtb_Add1_l = inverseIMU_gain[k + 3];
    i = 1;
    for (j = 0; j < 5; j++) {
      rtb_Add1_l -= localDW->IIRgyroz_states[memOffset + j] *
        localP->IIRgyroz_DenCoef[i];
      i++;
    }

    rtb_Add1_l /= localP->IIRgyroz_DenCoef[0];
    localDW->IIRgyroz_tmp[k] = rtb_Add1_l;
    rtb_r = localP->IIRgyroz_NumCoef[0] * localDW->IIRgyroz_tmp[k];
    i = 1;
    for (j = 0; j < 5; j++) {
      rtb_r += localDW->IIRgyroz_states[memOffset + j] *
        localP->IIRgyroz_NumCoef[i];
      i++;
    }

    rtb_Dk1uk1[k] = rtb_r;
  }

  /* End of DiscreteFilter: '<S69>/IIRgyroz' */

  /* SampleTimeMath: '<S119>/TSamp'
   *
   * About '<S119>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  absxk = rtb_Dk1uk1[0] * localP->TSamp_WtEt;
  t = rtb_Dk1uk1[1] * localP->TSamp_WtEt;

  /* Abs: '<S69>/Abs6' incorporates:
   *  Sum: '<S119>/Diff'
   *  UnitDelay: '<S119>/UD'
   */
  rtb_Dk1uk1_p = fabs(absxk - localDW->UD_DSTATE[0]);

  /* RelationalOperator: '<S120>/Compare' incorporates:
   *  Constant: '<S120>/Constant'
   */
  rtb_Compare_l = (rtb_Dk1uk1_p <= localP->maxdw1_const);

  /* Abs: '<S69>/Abs7' incorporates:
   *  Sum: '<S119>/Diff'
   *  UnitDelay: '<S119>/UD'
   */
  rtb_Dk1uk1_p = fabs(t - localDW->UD_DSTATE[1]);

  /* Logic: '<S69>/Logical Operator' incorporates:
   *  Constant: '<S121>/Constant'
   *  RelationalOperator: '<S121>/Compare'
   */
  rtb_LogicalOperator_o = (rtb_Compare_ii && rtb_Compare_du && rtb_Compare_po &&
    rtb_Compare_cz && rtb_Compare_l && (rtb_Dk1uk1_p <= localP->maxdw2_const));

  /* Abs: '<S69>/Abs4' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[3]);

  /* RelationalOperator: '<S123>/Compare' incorporates:
   *  Constant: '<S123>/Constant'
   */
  rtb_Compare_ii = (rtb_Dk1uk1_p <= localP->maxp2_const);

  /* Abs: '<S69>/Abs5' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[4]);

  /* Logic: '<S69>/Logical Operator1' incorporates:
   *  Constant: '<S125>/Constant'
   *  RelationalOperator: '<S125>/Compare'
   */
  rtb_Compare_po = (rtb_Compare_ii && (rtb_Dk1uk1_p <= localP->maxq2_const));

  /* Gain: '<S65>/opticalFlowToVelocity_gain' */
  rtb_opticalFlowToVelocity_gain[0] = localP->opticalFlowToVelocity_gain_Gain *
    localB->opticalFlow_datin[0];
  rtb_opticalFlowToVelocity_gain[1] = localP->opticalFlowToVelocity_gain_Gain *
    localB->opticalFlow_datin[1];
  rtb_opticalFlowToVelocity_gain[2] = localP->opticalFlowToVelocity_gain_Gain *
    localB->opticalFlow_datin[2];

  /* Abs: '<S69>/Abs8' incorporates:
   *  Delay: '<S65>/Delay'
   *  Sum: '<S69>/Add'
   */
  rtb_Dk1uk1_p = fabs(rtb_opticalFlowToVelocity_gain[0] -
                      localDW->Delay_DSTATE_b[0]);

  /* RelationalOperator: '<S128>/Compare' incorporates:
   *  Constant: '<S128>/Constant'
   */
  rtb_Compare_cz = (rtb_Dk1uk1_p <= localP->maxw3_const);

  /* Abs: '<S69>/Abs9' incorporates:
   *  Delay: '<S65>/Delay'
   *  Sum: '<S69>/Add'
   */
  rtb_Dk1uk1_p = fabs(rtb_opticalFlowToVelocity_gain[1] -
                      localDW->Delay_DSTATE_b[1]);

  /* RelationalOperator: '<S129>/Compare' incorporates:
   *  Constant: '<S129>/Constant'
   */
  rtb_Compare_l = (rtb_Dk1uk1_p <= localP->maxw4_const);

  /* Gain: '<S7>/invertzaxisGain' */
  rtb_invertzaxisGain = localP->invertzaxisGain_Gain * localB->sensordata_datin
    [6];

  /* Delay: '<S7>/Delay2' */
  rtb_Dk1uk1_p = localDW->Delay2_DSTATE;

  /* Saturate: '<S12>/SaturationSonar' */
  if (rtb_invertzaxisGain > -Drone_Compensator_P->quadEDT.altSenor_min) {
    rtb_TmpSignalConversionAtSFun_0 = -Drone_Compensator_P->quadEDT.altSenor_min;
  } else if (rtb_invertzaxisGain < localP->SaturationSonar_LowerSat) {
    rtb_TmpSignalConversionAtSFun_0 = localP->SaturationSonar_LowerSat;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_invertzaxisGain;
  }

  /* Sum: '<S12>/Add' incorporates:
   *  Saturate: '<S12>/SaturationSonar'
   */
  rtb_Ckxhatkk1_e = rtb_Dk1uk1_p - rtb_TmpSignalConversionAtSFun_0;

  /* Abs: '<S12>/Absestdiff' */
  rtb_Ckxhatkk1_e = fabs(rtb_Ckxhatkk1_e);

  /* RelationalOperator: '<S64>/Compare' incorporates:
   *  Constant: '<S64>/Constant'
   */
  rtb_Compare_ii = (rtb_Ckxhatkk1_e <= localP->outlierJump_const);

  /* Gain: '<S7>/prsToAlt_gain' incorporates:
   *  Sum: '<S10>/Sum2'
   */
  rtb_altfrompress = 1.0 / Drone_Compensator_P->quadEDT.altToPrs_gain *
    (localB->sensordata_datin[7] - localB->sensordataCalib_datin[6]);

  /* DiscreteFilter: '<S12>/pressureFilter_IIR' */
  pressureFilter_IIR_tmp = rtb_altfrompress;
  i = 1;
  for (j = 0; j < 5; j++) {
    pressureFilter_IIR_tmp -= localP->pressureFilter_IIR_DenCoef[i] *
      localDW->pressureFilter_IIR_states[j];
    i++;
  }

  pressureFilter_IIR_tmp /= localP->pressureFilter_IIR_DenCoef[0];
  rtb_r = localP->pressureFilter_IIR_NumCoef[0] * pressureFilter_IIR_tmp;
  i = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += localP->pressureFilter_IIR_NumCoef[i] *
      localDW->pressureFilter_IIR_states[j];
    i++;
  }

  rtb_Ckxhatkk1_e = rtb_r;

  /* End of DiscreteFilter: '<S12>/pressureFilter_IIR' */

  /* Sum: '<S12>/Add1' */
  rtb_Ckxhatkk1_e -= rtb_Dk1uk1_p;

  /* Abs: '<S12>/Absestdiff1' */
  rtb_Ckxhatkk1_e = fabs(rtb_Ckxhatkk1_e);

  /* RelationalOperator: '<S62>/Compare' incorporates:
   *  Constant: '<S62>/Constant'
   */
  rtb_Compare_du = (rtb_Ckxhatkk1_e >= localP->currentEstimateVeryOffFromPress);

  /* DiscreteFilter: '<S12>/soonarFilter_IIR' */
  soonarFilter_IIR_tmp = rtb_invertzaxisGain;
  i = 1;
  for (j = 0; j < 5; j++) {
    soonarFilter_IIR_tmp -= localP->soonarFilter_IIR_DenCoef[i] *
      localDW->soonarFilter_IIR_states[j];
    i++;
  }

  soonarFilter_IIR_tmp /= localP->soonarFilter_IIR_DenCoef[0];
  rtb_r = localP->soonarFilter_IIR_NumCoef[0] * soonarFilter_IIR_tmp;
  i = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += localP->soonarFilter_IIR_NumCoef[i] *
      localDW->soonarFilter_IIR_states[j];
    i++;
  }

  rtb_Ckxhatkk1_e = rtb_r;

  /* End of DiscreteFilter: '<S12>/soonarFilter_IIR' */

  /* Sum: '<S12>/Add2' */
  rtb_Ckxhatkk1_e -= rtb_Dk1uk1_p;

  /* Abs: '<S12>/Absestdiff2' */
  rtb_Ckxhatkk1_e = fabs(rtb_Ckxhatkk1_e);

  /* Logic: '<S12>/nicemeasurementor newupdateneeded' incorporates:
   *  Constant: '<S61>/Constant'
   *  Constant: '<S63>/Constant'
   *  Logic: '<S12>/findingoutliers'
   *  Logic: '<S12>/newupdateneeded'
   *  RelationalOperator: '<S61>/Compare'
   *  RelationalOperator: '<S63>/Compare'
   */
  rtb_nicemeasurementornewupdaten = ((rtb_Compare_ii && (rtb_invertzaxisGain <
    -Drone_Compensator_P->quadEDT.altSenor_min)) || (rtb_Compare_du &&
    (rtb_Ckxhatkk1_e >= localP->currentStateVeryOffsonarflt_con)));

  /* MATLAB Function: '<S7>/trafo_BodytoWorld_trans' */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorAltitude/trafo_BodytoWorld_trans': '<S14>:1' */
  /* '<S14>:1:2' */
  /* '<S14>:1:3' */
  /* '<S14>:1:4' */
  /* BBF > Inertial rotation matrix */
  /* '<S14>:1:6' */
  /* '<S14>:1:10' */
  tmp_1[0] = cos(localB->orient_estimout[1]) * cos(localB->orient_estimout[0]);
  tmp_1[3] = sin(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    cos(localB->orient_estimout[0]) - cos(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_1[6] = cos(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    cos(localB->orient_estimout[0]) + sin(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_1[1] = cos(localB->orient_estimout[1]) * sin(localB->orient_estimout[0]);
  tmp_1[4] = sin(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[0]) + cos(localB->orient_estimout[2]) * cos
    (localB->orient_estimout[0]);
  tmp_1[7] = cos(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[0]) - sin(localB->orient_estimout[2]) * cos
    (localB->orient_estimout[0]);
  tmp_1[2] = -sin(localB->orient_estimout[1]);
  tmp_1[5] = sin(localB->orient_estimout[2]) * cos(localB->orient_estimout[1]);
  tmp_1[8] = cos(localB->orient_estimout[2]) * cos(localB->orient_estimout[1]);

  /* Sum: '<S7>/Sum' incorporates:
   *  Constant: '<S7>/gravity'
   *  MATLAB Function: '<S7>/trafo_BodytoWorld_trans'
   */
  for (i = 0; i < 3; i++) {
    rtb_Sum[i] = ((tmp_1[i + 3] * localB->FIR_IMUaccel[1] + tmp_1[i] *
                   localB->FIR_IMUaccel[0]) + tmp_1[i + 6] *
                  localB->FIR_IMUaccel[2]) + localP->gravity_Value[i];
  }

  /* End of Sum: '<S7>/Sum' */

  /* Delay: '<S11>/MemoryX' incorporates:
   *  Constant: '<S11>/X0'
   *  Constant: '<S13>/Constant'
   *  RelationalOperator: '<S13>/Compare'
   */
  if (rtb_Dk1uk1_p > localP->outlierBelowFloor_const) {
    localDW->icLoad = 1U;
  }

  if (localDW->icLoad != 0) {
    localDW->MemoryX_DSTATE[0] = localP->X0_Value[0];
    localDW->MemoryX_DSTATE[1] = localP->X0_Value[1];
  }

  /* Outputs for Atomic SubSystem: '<S11>/UseCurrentEstimator' */
  /* Outputs for Enabled SubSystem: '<S39>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S60>/Enable'
   */
  if (rtb_nicemeasurementornewupdaten) {
    /* Sum: '<S60>/Add1' incorporates:
     *  Constant: '<S11>/C'
     *  Constant: '<S11>/D'
     *  Delay: '<S11>/MemoryX'
     *  Product: '<S60>/Product'
     *  Product: '<S60>/Product1'
     */
    rtb_Add1_l = (rtb_invertzaxisGain - (localP->C_Value[0] *
      localDW->MemoryX_DSTATE[0] + localP->C_Value[1] * localDW->MemoryX_DSTATE
      [1])) - localP->D_Value * rtb_Sum[2];

    /* Product: '<S60>/Product2' incorporates:
     *  Constant: '<S16>/KalmanGainM'
     */
    localB->Product2[0] = localP->KalmanGainM_Value_p[0] * rtb_Add1_l;
    localB->Product2[1] = localP->KalmanGainM_Value_p[1] * rtb_Add1_l;
  }

  /* End of Outputs for SubSystem: '<S39>/Enabled Subsystem' */

  /* Reshape: '<S11>/Reshapexhat' incorporates:
   *  Delay: '<S11>/MemoryX'
   *  Sum: '<S39>/Add'
   */
  localB->Reshapexhat[0] = localB->Product2[0] + localDW->MemoryX_DSTATE[0];
  localB->Reshapexhat[1] = localB->Product2[1] + localDW->MemoryX_DSTATE[1];

  /* End of Outputs for SubSystem: '<S11>/UseCurrentEstimator' */

  /* Logic: '<S69>/Logical Operator3' incorporates:
   *  Constant: '<S130>/Constant'
   *  Logic: '<S69>/Logical Operator2'
   *  RelationalOperator: '<S130>/Compare'
   */
  rtb_LogicalOperator3_j = ((rtb_LogicalOperator_o || rtb_Compare_po) &&
    rtb_Compare_cz && rtb_Compare_l && (localB->Reshapexhat[0] <=
    localP->minHeightforOF_const));

  /* Logic: '<S67>/Logical Operator' incorporates:
   *  Constant: '<S71>/Constant'
   *  Constant: '<S72>/Constant'
   *  RelationalOperator: '<S71>/Compare'
   *  RelationalOperator: '<S72>/Compare'
   */
  rtb_Compare_ii = ((rtb_opticalFlowToVelocity_gain[0] !=
                     localP->donotuseaccifopticalflowneverav) ||
                    (rtb_opticalFlowToVelocity_gain[1] !=
                     localP->donotuseaccifopticalflownever_g));

  /* RelationalOperator: '<S70>/Compare' incorporates:
   *  Constant: '<S70>/Constant'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/trafo_World2Body_trans': '<S73>:1' */
  /* '<S73>:1:3' */
  /* '<S73>:1:4' */
  /* '<S73>:1:5' */
  /* '<S73>:1:7' */
  /* '<S73>:1:11' */
  rtb_Compare_du = (localB->Reshapexhat[0] <=
                    localP->DeactivateAccelerationIfOFisnot);

  /* MATLAB Function: '<S67>/trafo_World2Body_trans' */
  tmp_2[0] = cos(localB->orient_estimout[1]) * cos(localB->orient_estimout[0]);
  tmp_2[3] = cos(localB->orient_estimout[1]) * sin(localB->orient_estimout[0]);
  tmp_2[6] = -sin(localB->orient_estimout[1]);
  tmp_2[1] = cos(localB->orient_estimout[0]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[2]) - cos(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_2[4] = sin(localB->orient_estimout[1]) * sin(localB->orient_estimout[2]) *
    sin(localB->orient_estimout[0]) + cos(localB->orient_estimout[2]) * cos
    (localB->orient_estimout[0]);
  tmp_2[7] = cos(localB->orient_estimout[1]) * sin(localB->orient_estimout[2]);
  tmp_2[2] = cos(localB->orient_estimout[2]) * cos(localB->orient_estimout[0]) *
    sin(localB->orient_estimout[1]) + sin(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_2[5] = cos(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[0]) - cos(localB->orient_estimout[0]) * sin
    (localB->orient_estimout[2]);
  tmp_2[8] = cos(localB->orient_estimout[1]) * cos(localB->orient_estimout[2]);

  /* Sum: '<S67>/Add' incorporates:
   *  Constant: '<S67>/gravity'
   *  Gain: '<S67>/gainaccinput'
   *  MATLAB Function: '<S67>/trafo_World2Body_trans'
   */
  for (i = 0; i < 3; i++) {
    rtb_vel_world[i] = localB->FIR_IMUaccel[i] - ((tmp_2[i + 3] *
      localP->gravity_Value_b[1] + tmp_2[i] * localP->gravity_Value_b[0]) +
      tmp_2[i + 6] * localP->gravity_Value_b[2]);
  }

  /* End of Sum: '<S67>/Add' */

  /* Product: '<S67>/Product' incorporates:
   *  Gain: '<S67>/gainaccinput'
   */
  rtb_Dk1uk1[0] = localP->gainaccinput_Gain * rtb_vel_world[0] * (real_T)
    rtb_Compare_ii * (real_T)rtb_Compare_du;
  rtb_Dk1uk1[1] = localP->gainaccinput_Gain * rtb_vel_world[1] * (real_T)
    rtb_Compare_ii * (real_T)rtb_Compare_du;

  /* Delay: '<S68>/MemoryX' incorporates:
   *  Constant: '<S68>/X0'
   */
  if (localDW->icLoad_l != 0) {
    localDW->MemoryX_DSTATE_g[0] = localP->X0_Value_j[0];
    localDW->MemoryX_DSTATE_g[1] = localP->X0_Value_j[1];
  }

  rtb_Add1[0] = localDW->MemoryX_DSTATE_g[0];
  rtb_Add1[1] = localDW->MemoryX_DSTATE_g[1];

  /* Outputs for Atomic SubSystem: '<S68>/UseCurrentEstimator' */

  /* Constant: '<S74>/KalmanGainM' incorporates:
   *  Constant: '<S68>/C'
   *  Constant: '<S68>/D'
   */
  Drone_Compe_UseCurrentEstimator(rtb_LogicalOperator3_j,
    localP->KalmanGainM_Value_n, rtb_Dk1uk1, &rtb_opticalFlowToVelocity_gain[0],
    localP->C_Value_j, localP->D_Value_b, rtb_Add1,
    &localB->UseCurrentEstimator_b);

  /* End of Outputs for SubSystem: '<S68>/UseCurrentEstimator' */

  /* Reshape: '<S68>/Reshapexhat' */
  localB->Reshapexhat_o[0] = localB->UseCurrentEstimator_b.Add[0];
  localB->Reshapexhat_o[1] = localB->UseCurrentEstimator_b.Add[1];

  /* SignalConversion: '<S15>/TmpSignal ConversionAt SFunction Inport2' incorporates:
   *  Delay: '<S3>/Delay1'
   *  MATLAB Function: '<S7>/trafo_WorldToBody_trans'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorAltitude/trafo_WorldToBody_trans': '<S15>:1' */
  /* '<S15>:1:2' */
  /* '<S15>:1:3' */
  /* '<S15>:1:4' */
  /* '<S15>:1:7' */
  /* '<S15>:1:11' */
  rtb_TmpSignalConversionAtSFun_0 = localDW->Delay1_DSTATE[0];
  scale = localDW->Delay1_DSTATE[1];

  /* MATLAB Function: '<S7>/trafo_WorldToBody_trans' incorporates:
   *  SignalConversion: '<S15>/TmpSignal ConversionAt SFunction Inport2'
   */
  tmp_3[0] = cos(localB->orient_estimout[1]) * cos(localB->orient_estimout[0]);
  tmp_3[3] = cos(localB->orient_estimout[1]) * sin(localB->orient_estimout[0]);
  tmp_3[6] = -sin(localB->orient_estimout[1]);
  tmp_3[1] = cos(localB->orient_estimout[0]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[2]) - cos(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_3[4] = sin(localB->orient_estimout[1]) * sin(localB->orient_estimout[2]) *
    sin(localB->orient_estimout[0]) + cos(localB->orient_estimout[2]) * cos
    (localB->orient_estimout[0]);
  tmp_3[7] = cos(localB->orient_estimout[1]) * sin(localB->orient_estimout[2]);
  tmp_3[2] = cos(localB->orient_estimout[2]) * cos(localB->orient_estimout[0]) *
    sin(localB->orient_estimout[1]) + sin(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_3[5] = cos(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[0]) - cos(localB->orient_estimout[0]) * sin
    (localB->orient_estimout[2]);
  tmp_3[8] = cos(localB->orient_estimout[1]) * cos(localB->orient_estimout[2]);
  for (i = 0; i < 3; i++) {
    localB->acc_RS[i] = 0.0;
    localB->acc_RS[i] += tmp_3[i] * rtb_TmpSignalConversionAtSFun_0;
    localB->acc_RS[i] += tmp_3[i + 3] * scale;
    localB->acc_RS[i] += tmp_3[i + 6] * localB->Reshapexhat[1];
  }

  /* MATLAB Function: '<S66>/trafo_BodytoWorld_trans' incorporates:
   *  SignalConversion: '<S133>/TmpSignal ConversionAt SFunction Inport2'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/trafo_BodytoWorld_trans': '<S133>:1' */
  /* '<S133>:1:2' */
  /* '<S133>:1:3' */
  /* '<S133>:1:4' */
  /* BBF > Inertial rotation matrix */
  /* '<S133>:1:6' */
  /* '<S133>:1:10' */
  tmp_4[0] = cos(localB->orient_estimout[1]) * cos(localB->orient_estimout[0]);
  tmp_4[3] = sin(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    cos(localB->orient_estimout[0]) - cos(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_4[6] = cos(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    cos(localB->orient_estimout[0]) + sin(localB->orient_estimout[2]) * sin
    (localB->orient_estimout[0]);
  tmp_4[1] = cos(localB->orient_estimout[1]) * sin(localB->orient_estimout[0]);
  tmp_4[4] = sin(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[0]) + cos(localB->orient_estimout[2]) * cos
    (localB->orient_estimout[0]);
  tmp_4[7] = cos(localB->orient_estimout[2]) * sin(localB->orient_estimout[1]) *
    sin(localB->orient_estimout[0]) - sin(localB->orient_estimout[2]) * cos
    (localB->orient_estimout[0]);
  tmp_4[2] = -sin(localB->orient_estimout[1]);
  tmp_4[5] = sin(localB->orient_estimout[2]) * cos(localB->orient_estimout[1]);
  tmp_4[8] = cos(localB->orient_estimout[2]) * cos(localB->orient_estimout[1]);
  for (i = 0; i < 3; i++) {
    rtb_vel_world[i] = tmp_4[i + 6] * localB->acc_RS[2] + (tmp_4[i + 3] *
      localB->Reshapexhat_o[1] + tmp_4[i] * localB->Reshapexhat_o[0]);
  }

  /* End of MATLAB Function: '<S66>/trafo_BodytoWorld_trans' */

  /* Delay: '<S131>/MemoryX' incorporates:
   *  Constant: '<S131>/X0'
   */
  if (localDW->icLoad_j != 0) {
    localDW->MemoryX_DSTATE_m[0] = localP->X0_Value_k[0];
    localDW->MemoryX_DSTATE_m[1] = localP->X0_Value_k[1];
  }

  rtb_Add1[0] = localDW->MemoryX_DSTATE_m[0];
  rtb_Add1[1] = localDW->MemoryX_DSTATE_m[1];

  /* Outputs for Atomic SubSystem: '<S131>/UseCurrentEstimator' */

  /* Constant: '<S134>/KalmanGainM' incorporates:
   *  Constant: '<S131>/C'
   *  Constant: '<S131>/D'
   */
  Drone_Compe_UseCurrentEstimator(rtb_LogicalOperator3,
    localP->KalmanGainM_Value, &rtb_vel_world[0], &localB->posVIS_datin[0],
    localP->C_Value_f, localP->D_Value_o, rtb_Add1,
    &localB->UseCurrentEstimator_g);

  /* End of Outputs for SubSystem: '<S131>/UseCurrentEstimator' */

  /* DiscreteIntegrator: '<S66>/SimplyIntegrateVelocity' */
  if (localB->controlModePosVSOrient_flagin &&
      (localDW->SimplyIntegrateVelocity_PrevRes <= 0)) {
    localDW->SimplyIntegrateVelocity_DSTATE[0] =
      localP->SimplyIntegrateVelocity_IC;
    localDW->SimplyIntegrateVelocity_DSTATE[1] =
      localP->SimplyIntegrateVelocity_IC;
  }

  if (localDW->SimplyIntegrateVelocity_DSTATE[0] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[0] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[0] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[0] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  if (localDW->SimplyIntegrateVelocity_DSTATE[1] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[1] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[1] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[1] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  /* Switch: '<S66>/UseIPPosSwitch' incorporates:
   *  DiscreteIntegrator: '<S66>/SimplyIntegrateVelocity'
   */
  if (localB->usePosVIS_flagin > localP->UseIPPosSwitch_Threshold) {
    localB->UseIPPosSwitch[0] = localB->UseCurrentEstimator_g.Add[0];
    localB->UseIPPosSwitch[1] = localB->UseCurrentEstimator_g.Add[1];
  } else {
    localB->UseIPPosSwitch[0] = localDW->SimplyIntegrateVelocity_DSTATE[0];
    localB->UseIPPosSwitch[1] = localDW->SimplyIntegrateVelocity_DSTATE[1];
  }

  /* End of Switch: '<S66>/UseIPPosSwitch' */

  /* Outputs for Atomic SubSystem: '<S1>/ControllerPID' */
  Drone_Compensator_ControllerPID(rtu_pos_refin, localB->takeoff_flag,
    rtu_orient_refin, localB->controlModePosVSOrient_flagin,
    localB->orient_estimout, localB->dorient_estimout, localB->acc_RS[2],
    localB->Reshapexhat[0], localB->Reshapexhat_o, localB->UseIPPosSwitch,
    &localB->ControllerPID, &localDW->ControllerPID,
    (P_ControllerPID_Drone_Compens_T *)&localP->ControllerPID,
    Drone_Compensator_P);

  /* End of Outputs for SubSystem: '<S1>/ControllerPID' */

  /* Bias: '<S7>/Bias' */
  rtb_Ckxhatkk1_e = localB->Reshapexhat[0] + localP->Bias_Bias;

  /* Bias: '<S7>/Bias1' */
  rtb_Dk1uk1_p = localB->Reshapexhat[0] + localP->Bias1_Bias;

  /* Product: '<S34>/C[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S11>/C'
   *  Delay: '<S11>/MemoryX'
   */
  rtb_Ckxhatkk1_e = localP->C_Value[0] * localDW->MemoryX_DSTATE[0] +
    localP->C_Value[1] * localDW->MemoryX_DSTATE[1];

  /* Product: '<S34>/D[k-1]*u[k-1]' incorporates:
   *  Constant: '<S11>/D'
   */
  rtb_Dk1uk1_p = localP->D_Value * rtb_Sum[2];

  /* Outputs for Enabled SubSystem: '<S34>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S59>/Enable'
   */
  if (rtb_nicemeasurementornewupdaten) {
    /* Sum: '<S59>/Sum' incorporates:
     *  Sum: '<S34>/Add1'
     */
    rtb_r = rtb_invertzaxisGain - (rtb_Ckxhatkk1_e + rtb_Dk1uk1_p);

    /* Product: '<S59>/Product3' incorporates:
     *  Constant: '<S16>/KalmanGainL'
     */
    localB->Product3[0] = localP->KalmanGainL_Value[0] * rtb_r;
    localB->Product3[1] = localP->KalmanGainL_Value[1] * rtb_r;
  }

  /* End of Outputs for SubSystem: '<S34>/MeasurementUpdate' */

  /* Sum: '<S92>/Add1' incorporates:
   *  Constant: '<S68>/C'
   *  Constant: '<S68>/D'
   *  Delay: '<S68>/MemoryX'
   *  Product: '<S92>/C[k]*xhat[k|k-1]'
   *  Product: '<S92>/D[k-1]*u[k-1]'
   */
  rtb_Add1[0] = (localP->C_Value_j[0] * localDW->MemoryX_DSTATE_g[0] +
                 localP->C_Value_j[2] * localDW->MemoryX_DSTATE_g[1]) +
    (localP->D_Value_b[0] * rtb_Dk1uk1[0] + localP->D_Value_b[2] * rtb_Dk1uk1[1]);
  rtb_Add1[1] = (localP->C_Value_j[1] * localDW->MemoryX_DSTATE_g[0] +
                 localP->C_Value_j[3] * localDW->MemoryX_DSTATE_g[1]) +
    (localP->D_Value_b[1] * rtb_Dk1uk1[0] + localP->D_Value_b[3] * rtb_Dk1uk1[1]);

  /* Outputs for Enabled SubSystem: '<S92>/MeasurementUpdate' */

  /* Constant: '<S74>/KalmanGainL' */
  Drone_Compens_MeasurementUpdate(rtb_LogicalOperator3_j,
    localP->KalmanGainL_Value_p, &rtb_opticalFlowToVelocity_gain[0], rtb_Add1,
    &localB->MeasurementUpdate_h);

  /* End of Outputs for SubSystem: '<S92>/MeasurementUpdate' */

  /* Product: '<S92>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S68>/A'
   *  Delay: '<S68>/MemoryX'
   *  Sum: '<S92>/Add'
   */
  rtb_TmpSignalConversionAtSFun_0 = localP->A_Value_m[1] *
    localDW->MemoryX_DSTATE_g[0] + localP->A_Value_m[3] *
    localDW->MemoryX_DSTATE_g[1];

  /* Update for Delay: '<S68>/MemoryX' incorporates:
   *  Constant: '<S68>/A'
   *  Constant: '<S68>/B'
   *  Delay: '<S68>/MemoryX'
   *  Product: '<S92>/A[k]*xhat[k|k-1]'
   *  Product: '<S92>/B[k]*u[k]'
   *  Sum: '<S92>/Add'
   */
  localDW->MemoryX_DSTATE_g[0] = ((localP->B_Value_b[0] * rtb_Dk1uk1[0] +
    localP->B_Value_b[2] * rtb_Dk1uk1[1]) + (localP->A_Value_m[0] *
    localDW->MemoryX_DSTATE_g[0] + localP->A_Value_m[2] *
    localDW->MemoryX_DSTATE_g[1])) + localB->MeasurementUpdate_h.Product3[0];
  localDW->MemoryX_DSTATE_g[1] = ((localP->B_Value_b[1] * rtb_Dk1uk1[0] +
    localP->B_Value_b[3] * rtb_Dk1uk1[1]) + rtb_TmpSignalConversionAtSFun_0) +
    localB->MeasurementUpdate_h.Product3[1];

  /* Sum: '<S152>/Add1' incorporates:
   *  Constant: '<S131>/C'
   *  Constant: '<S131>/D'
   *  Delay: '<S131>/MemoryX'
   *  Product: '<S152>/C[k]*xhat[k|k-1]'
   *  Product: '<S152>/D[k-1]*u[k-1]'
   */
  rtb_Add1[0] = (localP->C_Value_f[0] * localDW->MemoryX_DSTATE_m[0] +
                 localP->C_Value_f[2] * localDW->MemoryX_DSTATE_m[1]) +
    (localP->D_Value_o[0] * rtb_vel_world[0] + localP->D_Value_o[2] *
     rtb_vel_world[1]);
  rtb_Add1[1] = (localP->C_Value_f[1] * localDW->MemoryX_DSTATE_m[0] +
                 localP->C_Value_f[3] * localDW->MemoryX_DSTATE_m[1]) +
    (localP->D_Value_o[1] * rtb_vel_world[0] + localP->D_Value_o[3] *
     rtb_vel_world[1]);

  /* Outputs for Enabled SubSystem: '<S152>/MeasurementUpdate' */

  /* Constant: '<S134>/KalmanGainL' */
  Drone_Compens_MeasurementUpdate(rtb_LogicalOperator3,
    localP->KalmanGainL_Value_o, &localB->posVIS_datin[0], rtb_Add1,
    &localB->MeasurementUpdate_o);

  /* End of Outputs for SubSystem: '<S152>/MeasurementUpdate' */

  /* Inport: '<S1>/batteryStatus_datin' */
  localB->batteryStatus_datin[0] = rtu_batteryStatus_datin[0];
  localB->batteryStatus_datin[1] = rtu_batteryStatus_datin[1];

  /* Update for DiscreteFir: '<S10>/FIR_IMUaccel' */
  /* Update circular buffer index */
  localDW->FIR_IMUaccel_circBuf--;
  if (localDW->FIR_IMUaccel_circBuf < 0) {
    localDW->FIR_IMUaccel_circBuf = 4;
  }

  /* Update circular buffer */
  localDW->FIR_IMUaccel_states[localDW->FIR_IMUaccel_circBuf] = inverseIMU_gain
    [0];
  localDW->FIR_IMUaccel_states[localDW->FIR_IMUaccel_circBuf + 5] =
    inverseIMU_gain[1];
  localDW->FIR_IMUaccel_states[localDW->FIR_IMUaccel_circBuf + 10] =
    inverseIMU_gain[2];

  /* End of Update for DiscreteFir: '<S10>/FIR_IMUaccel' */

  /* Update for DiscreteFilter: '<S10>/IIR_IMUgyro_r' */
  localDW->IIR_IMUgyro_r_states[4] = localDW->IIR_IMUgyro_r_states[3];
  localDW->IIR_IMUgyro_r_states[3] = localDW->IIR_IMUgyro_r_states[2];
  localDW->IIR_IMUgyro_r_states[2] = localDW->IIR_IMUgyro_r_states[1];
  localDW->IIR_IMUgyro_r_states[1] = localDW->IIR_IMUgyro_r_states[0];
  localDW->IIR_IMUgyro_r_states[0] = IIR_IMUgyro_r_tmp;

  /* Update for Delay: '<S66>/Delay' */
  localDW->Delay_DSTATE[0] = localB->UseCurrentEstimator_g.Add[0];
  localDW->Delay_DSTATE[1] = localB->UseCurrentEstimator_g.Add[1];

  /* Update for DiscreteFilter: '<S69>/IIRgyroz' */
  for (k = 0; k < 2; k++) {
    memOffset = k * 5;
    localDW->IIRgyroz_states[memOffset + 4] = localDW->IIRgyroz_states[memOffset
      + 3];
    localDW->IIRgyroz_states[memOffset + 3] = localDW->IIRgyroz_states[memOffset
      + 2];
    localDW->IIRgyroz_states[memOffset + 2] = localDW->IIRgyroz_states[memOffset
      + 1];
    localDW->IIRgyroz_states[memOffset + 1] = localDW->IIRgyroz_states[memOffset];
    localDW->IIRgyroz_states[memOffset] = localDW->IIRgyroz_tmp[k];
  }

  /* End of Update for DiscreteFilter: '<S69>/IIRgyroz' */

  /* Update for UnitDelay: '<S119>/UD' */
  localDW->UD_DSTATE[0] = absxk;
  localDW->UD_DSTATE[1] = t;

  /* Update for Delay: '<S65>/Delay' */
  localDW->Delay_DSTATE_b[0] = localB->Reshapexhat_o[0];
  localDW->Delay_DSTATE_b[1] = localB->Reshapexhat_o[1];

  /* Update for Delay: '<S7>/Delay2' */
  localDW->Delay2_DSTATE = localB->Reshapexhat[0];

  /* Update for DiscreteFilter: '<S12>/pressureFilter_IIR' */
  localDW->pressureFilter_IIR_states[4] = localDW->pressureFilter_IIR_states[3];
  localDW->pressureFilter_IIR_states[3] = localDW->pressureFilter_IIR_states[2];
  localDW->pressureFilter_IIR_states[2] = localDW->pressureFilter_IIR_states[1];
  localDW->pressureFilter_IIR_states[1] = localDW->pressureFilter_IIR_states[0];
  localDW->pressureFilter_IIR_states[0] = pressureFilter_IIR_tmp;

  /* Update for DiscreteFilter: '<S12>/soonarFilter_IIR' */
  localDW->soonarFilter_IIR_states[4] = localDW->soonarFilter_IIR_states[3];
  localDW->soonarFilter_IIR_states[3] = localDW->soonarFilter_IIR_states[2];
  localDW->soonarFilter_IIR_states[2] = localDW->soonarFilter_IIR_states[1];
  localDW->soonarFilter_IIR_states[1] = localDW->soonarFilter_IIR_states[0];
  localDW->soonarFilter_IIR_states[0] = soonarFilter_IIR_tmp;

  /* Update for Delay: '<S11>/MemoryX' */
  localDW->icLoad = 0U;

  /* Product: '<S34>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S11>/A'
   *  Delay: '<S11>/MemoryX'
   *  Sum: '<S34>/Add'
   */
  rtb_TmpSignalConversionAtSFun_0 = localP->A_Value[1] * localDW->
    MemoryX_DSTATE[0] + localP->A_Value[3] * localDW->MemoryX_DSTATE[1];

  /* Update for Delay: '<S11>/MemoryX' incorporates:
   *  Constant: '<S11>/A'
   *  Constant: '<S11>/B'
   *  Delay: '<S11>/MemoryX'
   *  Product: '<S34>/A[k]*xhat[k|k-1]'
   *  Product: '<S34>/B[k]*u[k]'
   *  Sum: '<S34>/Add'
   */
  localDW->MemoryX_DSTATE[0] = ((localP->A_Value[0] * localDW->MemoryX_DSTATE[0]
    + localP->A_Value[2] * localDW->MemoryX_DSTATE[1]) + localP->B_Value[0] *
    rtb_Sum[2]) + localB->Product3[0];
  localDW->MemoryX_DSTATE[1] = (localP->B_Value[1] * rtb_Sum[2] +
    rtb_TmpSignalConversionAtSFun_0) + localB->Product3[1];

  /* Update for Delay: '<S68>/MemoryX' */
  localDW->icLoad_l = 0U;

  /* Update for Delay: '<S3>/Delay1' */
  localDW->Delay1_DSTATE[0] = localB->Reshapexhat_o[0];
  localDW->Delay1_DSTATE[1] = localB->Reshapexhat_o[1];

  /* Update for Delay: '<S131>/MemoryX' */
  localDW->icLoad_j = 0U;

  /* Product: '<S152>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S131>/A'
   *  Delay: '<S131>/MemoryX'
   *  Sum: '<S152>/Add'
   */
  rtb_TmpSignalConversionAtSFun_0 = localP->A_Value_g[1] *
    localDW->MemoryX_DSTATE_m[0] + localP->A_Value_g[3] *
    localDW->MemoryX_DSTATE_m[1];

  /* Update for Delay: '<S131>/MemoryX' incorporates:
   *  Constant: '<S131>/A'
   *  Constant: '<S131>/B'
   *  Delay: '<S131>/MemoryX'
   *  Product: '<S152>/A[k]*xhat[k|k-1]'
   *  Product: '<S152>/B[k]*u[k]'
   *  Sum: '<S152>/Add'
   */
  localDW->MemoryX_DSTATE_m[0] = ((localP->B_Value_a[0] * rtb_vel_world[0] +
    localP->B_Value_a[2] * rtb_vel_world[1]) + (localP->A_Value_g[0] *
    localDW->MemoryX_DSTATE_m[0] + localP->A_Value_g[2] *
    localDW->MemoryX_DSTATE_m[1])) + localB->MeasurementUpdate_o.Product3[0];
  localDW->MemoryX_DSTATE_m[1] = ((localP->B_Value_a[1] * rtb_vel_world[0] +
    localP->B_Value_a[3] * rtb_vel_world[1]) + rtb_TmpSignalConversionAtSFun_0)
    + localB->MeasurementUpdate_o.Product3[1];

  /* Update for DiscreteIntegrator: '<S66>/SimplyIntegrateVelocity' */
  localDW->SimplyIntegrateVelocity_DSTATE[0] +=
    localP->SimplyIntegrateVelocity_gainval * rtb_vel_world[0];
  localDW->SimplyIntegrateVelocity_DSTATE[1] +=
    localP->SimplyIntegrateVelocity_gainval * rtb_vel_world[1];
  if (localDW->SimplyIntegrateVelocity_DSTATE[0] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[0] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[0] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[0] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  if (localDW->SimplyIntegrateVelocity_DSTATE[1] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[1] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[1] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[1] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  localDW->SimplyIntegrateVelocity_PrevRes = (int8_T)
    localB->controlModePosVSOrient_flagin;

  /* End of Update for DiscreteIntegrator: '<S66>/SimplyIntegrateVelocity' */
}

/* Model step function */
void Drone_Compensator_step(RT_MODEL_Drone_Compensator_T *const
  Drone_Compensator_M, boolean_T
  Drone_Compensator_U_controlModePosVSOrient_flagin, real_T
  Drone_Compensator_U_pos_refin[3], boolean_T Drone_Compensator_U_takeoff_flag,
  real_T Drone_Compensator_U_orient_refin[3], real_T Drone_Compensator_U_ddx,
  real_T Drone_Compensator_U_ddy, real_T Drone_Compensator_U_ddz, real_T
  Drone_Compensator_U_p, real_T Drone_Compensator_U_q, real_T
  Drone_Compensator_U_r, real_T Drone_Compensator_U_altitude_sonar, real_T
  Drone_Compensator_U_prs, real_T Drone_Compensator_U_opticalFlow_datin[3],
  real_T Drone_Compensator_U_sensordataCalib_datin[7], real_T
  Drone_Compensator_U_posVIS_datin[4], real_T
  Drone_Compensator_U_usePosVIS_flagin, real_T
  Drone_Compensator_U_batteryStatus_datin[2], real_T
  Drone_Compensator_Y_motors_refout[4], real_T *Drone_Compensator_Y_X, real_T
  *Drone_Compensator_Y_Y, real_T *Drone_Compensator_Y_Z, real_T
  *Drone_Compensator_Y_yaw, real_T *Drone_Compensator_Y_pitch, real_T
  *Drone_Compensator_Y_roll, real_T *Drone_Compensator_Y_dx, real_T
  *Drone_Compensator_Y_dy, real_T *Drone_Compensator_Y_dz, real_T
  *Drone_Compensator_Y_pb, real_T *Drone_Compensator_Y_qb, real_T
  *Drone_Compensator_Y_rb, boolean_T
  *Drone_Compensator_Y_controlModePosVSOrient_flagout, real_T
  Drone_Compensator_Y_pose_refout[6], real_T *Drone_Compensator_Y_ddxb, real_T
  *Drone_Compensator_Y_ddyb, real_T *Drone_Compensator_Y_ddzb, real_T
  *Drone_Compensator_Y_pa, real_T *Drone_Compensator_Y_qa, real_T
  *Drone_Compensator_Y_ra, real_T *Drone_Compensator_Y_altitude_sonarb, real_T
  *Drone_Compensator_Y_prsb, real_T Drone_Compensator_Y_opticalFlow_datout[3],
  real_T Drone_Compensator_Y_sensordataCalib_datout[7], real_T
  Drone_Compensator_Y_posVIS_datout[4], real_T
  *Drone_Compensator_Y_usePosVIS_flagout, real_T
  Drone_Compensator_Y_batteryStatus_datout[2], boolean_T
  *Drone_Compensator_Y_takeoff_flagout)
{
  P_Drone_Compensator_T *Drone_Compensator_P = ((P_Drone_Compensator_T *)
    Drone_Compensator_M->ModelData.defaultParam);
  B_Drone_Compensator_T *Drone_Compensator_B = ((B_Drone_Compensator_T *)
    Drone_Compensator_M->ModelData.blockIO);
  DW_Drone_Compensator_T *Drone_Compensator_DW = ((DW_Drone_Compensator_T *)
    Drone_Compensator_M->ModelData.dwork);
  int32_T i;

  /* Outputs for Atomic SubSystem: '<Root>/Drone_Compensator' */

  /* Inport: '<Root>/controlModePosVSOrient_flagin' incorporates:
   *  Inport: '<Root>/altitude_sonar'
   *  Inport: '<Root>/batteryStatus_datin'
   *  Inport: '<Root>/ddx'
   *  Inport: '<Root>/ddy'
   *  Inport: '<Root>/ddz'
   *  Inport: '<Root>/opticalFlow_datin'
   *  Inport: '<Root>/orient_refin'
   *  Inport: '<Root>/p'
   *  Inport: '<Root>/posVIS_datin'
   *  Inport: '<Root>/pos_refin'
   *  Inport: '<Root>/prs'
   *  Inport: '<Root>/q'
   *  Inport: '<Root>/r'
   *  Inport: '<Root>/sensordataCalib_datin'
   *  Inport: '<Root>/takeoff_flag'
   *  Inport: '<Root>/usePosVIS_flagin'
   */
  Drone_Compens_Drone_Compensator
    (Drone_Compensator_U_controlModePosVSOrient_flagin,
     Drone_Compensator_U_pos_refin, Drone_Compensator_U_takeoff_flag,
     Drone_Compensator_U_orient_refin, Drone_Compensator_U_ddx,
     Drone_Compensator_U_ddy, Drone_Compensator_U_ddz, Drone_Compensator_U_p,
     Drone_Compensator_U_q, Drone_Compensator_U_r,
     Drone_Compensator_U_altitude_sonar, Drone_Compensator_U_prs,
     Drone_Compensator_U_opticalFlow_datin,
     Drone_Compensator_U_sensordataCalib_datin, Drone_Compensator_U_posVIS_datin,
     Drone_Compensator_U_usePosVIS_flagin,
     Drone_Compensator_U_batteryStatus_datin,
     &Drone_Compensator_B->Drone_Compensator_d,
     &Drone_Compensator_DW->Drone_Compensator_d,
     (P_Drone_Compensator_Drone_Com_T *)
     &Drone_Compensator_P->Drone_Compensator_d, Drone_Compensator_P);

  /* End of Outputs for SubSystem: '<Root>/Drone_Compensator' */

  /* Outport: '<Root>/motors_refout' */
  Drone_Compensator_Y_motors_refout[0] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.Motordirections1[0];
  Drone_Compensator_Y_motors_refout[1] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.Motordirections1[1];
  Drone_Compensator_Y_motors_refout[2] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.Motordirections1[2];
  Drone_Compensator_Y_motors_refout[3] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.Motordirections1[3];

  /* Outport: '<Root>/X' */
  *Drone_Compensator_Y_X =
    Drone_Compensator_B->Drone_Compensator_d.UseIPPosSwitch[0];

  /* Outport: '<Root>/Y' */
  *Drone_Compensator_Y_Y =
    Drone_Compensator_B->Drone_Compensator_d.UseIPPosSwitch[1];

  /* Outport: '<Root>/Z' */
  *Drone_Compensator_Y_Z = Drone_Compensator_B->Drone_Compensator_d.Reshapexhat
    [0];

  /* Outport: '<Root>/yaw' */
  *Drone_Compensator_Y_yaw =
    Drone_Compensator_B->Drone_Compensator_d.orient_estimout[0];

  /* Outport: '<Root>/pitch' */
  *Drone_Compensator_Y_pitch =
    Drone_Compensator_B->Drone_Compensator_d.orient_estimout[1];

  /* Outport: '<Root>/roll' */
  *Drone_Compensator_Y_roll =
    Drone_Compensator_B->Drone_Compensator_d.orient_estimout[2];

  /* Outport: '<Root>/dx' */
  *Drone_Compensator_Y_dx =
    Drone_Compensator_B->Drone_Compensator_d.Reshapexhat_o[0];

  /* Outport: '<Root>/dy' */
  *Drone_Compensator_Y_dy =
    Drone_Compensator_B->Drone_Compensator_d.Reshapexhat_o[1];

  /* Outport: '<Root>/dz' */
  *Drone_Compensator_Y_dz = Drone_Compensator_B->Drone_Compensator_d.acc_RS[2];

  /* Outport: '<Root>/pb' */
  *Drone_Compensator_Y_pb =
    Drone_Compensator_B->Drone_Compensator_d.dorient_estimout[0];

  /* Outport: '<Root>/qb' */
  *Drone_Compensator_Y_qb =
    Drone_Compensator_B->Drone_Compensator_d.dorient_estimout[1];

  /* Outport: '<Root>/rb' */
  *Drone_Compensator_Y_rb =
    Drone_Compensator_B->Drone_Compensator_d.dorient_estimout[2];

  /* Outport: '<Root>/controlModePosVSOrient_flagout' */
  *Drone_Compensator_Y_controlModePosVSOrient_flagout =
    Drone_Compensator_B->Drone_Compensator_d.controlModePosVSOrient_flagin;

  /* Outport: '<Root>/pose_refout' */
  Drone_Compensator_Y_pose_refout[0] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.pos_ref[0];
  Drone_Compensator_Y_pose_refout[1] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.pos_ref[1];
  Drone_Compensator_Y_pose_refout[2] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.pos_ref[2];
  Drone_Compensator_Y_pose_refout[3] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.orient_ref[0];
  Drone_Compensator_Y_pose_refout[4] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.Switch_refAtt[0];
  Drone_Compensator_Y_pose_refout[5] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPID.Switch_refAtt[1];

  /* Outport: '<Root>/ddxb' */
  *Drone_Compensator_Y_ddxb =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[0];

  /* Outport: '<Root>/ddyb' */
  *Drone_Compensator_Y_ddyb =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[1];

  /* Outport: '<Root>/ddzb' */
  *Drone_Compensator_Y_ddzb =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[2];

  /* Outport: '<Root>/pa' */
  *Drone_Compensator_Y_pa =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[3];

  /* Outport: '<Root>/qa' */
  *Drone_Compensator_Y_qa =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[4];

  /* Outport: '<Root>/ra' */
  *Drone_Compensator_Y_ra =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[5];

  /* Outport: '<Root>/altitude_sonarb' */
  *Drone_Compensator_Y_altitude_sonarb =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[6];

  /* Outport: '<Root>/prsb' */
  *Drone_Compensator_Y_prsb =
    Drone_Compensator_B->Drone_Compensator_d.sensordata_datin[7];

  /* Outport: '<Root>/opticalFlow_datout' */
  Drone_Compensator_Y_opticalFlow_datout[0] =
    Drone_Compensator_B->Drone_Compensator_d.opticalFlow_datin[0];
  Drone_Compensator_Y_opticalFlow_datout[1] =
    Drone_Compensator_B->Drone_Compensator_d.opticalFlow_datin[1];
  Drone_Compensator_Y_opticalFlow_datout[2] =
    Drone_Compensator_B->Drone_Compensator_d.opticalFlow_datin[2];

  /* Outport: '<Root>/sensordataCalib_datout' */
  for (i = 0; i < 7; i++) {
    Drone_Compensator_Y_sensordataCalib_datout[i] =
      Drone_Compensator_B->Drone_Compensator_d.sensordataCalib_datin[i];
  }

  /* End of Outport: '<Root>/sensordataCalib_datout' */

  /* Outport: '<Root>/posVIS_datout' */
  Drone_Compensator_Y_posVIS_datout[0] =
    Drone_Compensator_B->Drone_Compensator_d.posVIS_datin[0];
  Drone_Compensator_Y_posVIS_datout[1] =
    Drone_Compensator_B->Drone_Compensator_d.posVIS_datin[1];
  Drone_Compensator_Y_posVIS_datout[2] =
    Drone_Compensator_B->Drone_Compensator_d.posVIS_datin[2];
  Drone_Compensator_Y_posVIS_datout[3] =
    Drone_Compensator_B->Drone_Compensator_d.posVIS_datin[3];

  /* Outport: '<Root>/usePosVIS_flagout' */
  *Drone_Compensator_Y_usePosVIS_flagout =
    Drone_Compensator_B->Drone_Compensator_d.usePosVIS_flagin;

  /* Outport: '<Root>/batteryStatus_datout' */
  Drone_Compensator_Y_batteryStatus_datout[0] =
    Drone_Compensator_B->Drone_Compensator_d.batteryStatus_datin[0];
  Drone_Compensator_Y_batteryStatus_datout[1] =
    Drone_Compensator_B->Drone_Compensator_d.batteryStatus_datin[1];

  /* Outport: '<Root>/takeoff_flagout' */
  *Drone_Compensator_Y_takeoff_flagout =
    Drone_Compensator_B->Drone_Compensator_d.takeoff_flag;

  /* Matfile logging */
  rt_UpdateTXYLogVars(Drone_Compensator_M->rtwLogInfo,
                      (&Drone_Compensator_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.005s, 0.0s] */
    if ((rtmGetTFinal(Drone_Compensator_M)!=-1) &&
        !((rtmGetTFinal(Drone_Compensator_M)-
           Drone_Compensator_M->Timing.taskTime0) >
          Drone_Compensator_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Drone_Compensator_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  Drone_Compensator_M->Timing.taskTime0 =
    (++Drone_Compensator_M->Timing.clockTick0) *
    Drone_Compensator_M->Timing.stepSize0;
}

/* Model initialize function */
void Drone_Compensator_initialize(RT_MODEL_Drone_Compensator_T *const
  Drone_Compensator_M, boolean_T
  *Drone_Compensator_U_controlModePosVSOrient_flagin, real_T
  Drone_Compensator_U_pos_refin[3], boolean_T *Drone_Compensator_U_takeoff_flag,
  real_T Drone_Compensator_U_orient_refin[3], real_T *Drone_Compensator_U_ddx,
  real_T *Drone_Compensator_U_ddy, real_T *Drone_Compensator_U_ddz, real_T
  *Drone_Compensator_U_p, real_T *Drone_Compensator_U_q, real_T
  *Drone_Compensator_U_r, real_T *Drone_Compensator_U_altitude_sonar, real_T
  *Drone_Compensator_U_prs, real_T Drone_Compensator_U_opticalFlow_datin[3],
  real_T Drone_Compensator_U_sensordataCalib_datin[7], real_T
  Drone_Compensator_U_posVIS_datin[4], real_T
  *Drone_Compensator_U_usePosVIS_flagin, real_T
  Drone_Compensator_U_batteryStatus_datin[2], real_T
  Drone_Compensator_Y_motors_refout[4], real_T *Drone_Compensator_Y_X, real_T
  *Drone_Compensator_Y_Y, real_T *Drone_Compensator_Y_Z, real_T
  *Drone_Compensator_Y_yaw, real_T *Drone_Compensator_Y_pitch, real_T
  *Drone_Compensator_Y_roll, real_T *Drone_Compensator_Y_dx, real_T
  *Drone_Compensator_Y_dy, real_T *Drone_Compensator_Y_dz, real_T
  *Drone_Compensator_Y_pb, real_T *Drone_Compensator_Y_qb, real_T
  *Drone_Compensator_Y_rb, boolean_T
  *Drone_Compensator_Y_controlModePosVSOrient_flagout, real_T
  Drone_Compensator_Y_pose_refout[6], real_T *Drone_Compensator_Y_ddxb, real_T
  *Drone_Compensator_Y_ddyb, real_T *Drone_Compensator_Y_ddzb, real_T
  *Drone_Compensator_Y_pa, real_T *Drone_Compensator_Y_qa, real_T
  *Drone_Compensator_Y_ra, real_T *Drone_Compensator_Y_altitude_sonarb, real_T
  *Drone_Compensator_Y_prsb, real_T Drone_Compensator_Y_opticalFlow_datout[3],
  real_T Drone_Compensator_Y_sensordataCalib_datout[7], real_T
  Drone_Compensator_Y_posVIS_datout[4], real_T
  *Drone_Compensator_Y_usePosVIS_flagout, real_T
  Drone_Compensator_Y_batteryStatus_datout[2], boolean_T
  *Drone_Compensator_Y_takeoff_flagout)
{
  P_Drone_Compensator_T *Drone_Compensator_P = ((P_Drone_Compensator_T *)
    Drone_Compensator_M->ModelData.defaultParam);
  DW_Drone_Compensator_T *Drone_Compensator_DW = ((DW_Drone_Compensator_T *)
    Drone_Compensator_M->ModelData.dwork);
  B_Drone_Compensator_T *Drone_Compensator_B = ((B_Drone_Compensator_T *)
    Drone_Compensator_M->ModelData.blockIO);

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  Drone_Compensator_P->Drone_Compensator_d.SaturationSonar_LowerSat = rtMinusInf;
  rtmSetTFinal(Drone_Compensator_M, -1);
  Drone_Compensator_M->Timing.stepSize0 = 0.005;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    Drone_Compensator_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Drone_Compensator_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Drone_Compensator_M->rtwLogInfo, (NULL));
    rtliSetLogT(Drone_Compensator_M->rtwLogInfo, "tout");
    rtliSetLogX(Drone_Compensator_M->rtwLogInfo, "");
    rtliSetLogXFinal(Drone_Compensator_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Drone_Compensator_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Drone_Compensator_M->rtwLogInfo, 2);
    rtliSetLogMaxRows(Drone_Compensator_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Drone_Compensator_M->rtwLogInfo, 1);

    /*
     * Set pointers to the data and signal info for each output
     */
    {
      static void * rt_LoggedOutputSignalPtrs[29];
      rt_LoggedOutputSignalPtrs[0] = &Drone_Compensator_Y_motors_refout[0];
      rt_LoggedOutputSignalPtrs[1] = &(*Drone_Compensator_Y_X);
      rt_LoggedOutputSignalPtrs[2] = &(*Drone_Compensator_Y_Y);
      rt_LoggedOutputSignalPtrs[3] = &(*Drone_Compensator_Y_Z);
      rt_LoggedOutputSignalPtrs[4] = &(*Drone_Compensator_Y_yaw);
      rt_LoggedOutputSignalPtrs[5] = &(*Drone_Compensator_Y_pitch);
      rt_LoggedOutputSignalPtrs[6] = &(*Drone_Compensator_Y_roll);
      rt_LoggedOutputSignalPtrs[7] = &(*Drone_Compensator_Y_dx);
      rt_LoggedOutputSignalPtrs[8] = &(*Drone_Compensator_Y_dy);
      rt_LoggedOutputSignalPtrs[9] = &(*Drone_Compensator_Y_dz);
      rt_LoggedOutputSignalPtrs[10] = &(*Drone_Compensator_Y_pb);
      rt_LoggedOutputSignalPtrs[11] = &(*Drone_Compensator_Y_qb);
      rt_LoggedOutputSignalPtrs[12] = &(*Drone_Compensator_Y_rb);
      rt_LoggedOutputSignalPtrs[13] =
        &(*Drone_Compensator_Y_controlModePosVSOrient_flagout);
      rt_LoggedOutputSignalPtrs[14] = &Drone_Compensator_Y_pose_refout[0];
      rt_LoggedOutputSignalPtrs[15] = &(*Drone_Compensator_Y_ddxb);
      rt_LoggedOutputSignalPtrs[16] = &(*Drone_Compensator_Y_ddyb);
      rt_LoggedOutputSignalPtrs[17] = &(*Drone_Compensator_Y_ddzb);
      rt_LoggedOutputSignalPtrs[18] = &(*Drone_Compensator_Y_pa);
      rt_LoggedOutputSignalPtrs[19] = &(*Drone_Compensator_Y_qa);
      rt_LoggedOutputSignalPtrs[20] = &(*Drone_Compensator_Y_ra);
      rt_LoggedOutputSignalPtrs[21] = &(*Drone_Compensator_Y_altitude_sonarb);
      rt_LoggedOutputSignalPtrs[22] = &(*Drone_Compensator_Y_prsb);
      rt_LoggedOutputSignalPtrs[23] = &Drone_Compensator_Y_opticalFlow_datout[0];
      rt_LoggedOutputSignalPtrs[24] =
        &Drone_Compensator_Y_sensordataCalib_datout[0];
      rt_LoggedOutputSignalPtrs[25] = &Drone_Compensator_Y_posVIS_datout[0];
      rt_LoggedOutputSignalPtrs[26] = &(*Drone_Compensator_Y_usePosVIS_flagout);
      rt_LoggedOutputSignalPtrs[27] = &Drone_Compensator_Y_batteryStatus_datout
        [0];
      rt_LoggedOutputSignalPtrs[28] = &(*Drone_Compensator_Y_takeoff_flagout);
      rtliSetLogYSignalPtrs(Drone_Compensator_M->rtwLogInfo, ((LogSignalPtrsType)
        rt_LoggedOutputSignalPtrs));
    }

    {
      static int_T rt_LoggedOutputWidths[] = {
        4,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        6,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        3,
        7,
        4,
        1,
        2,
        1
      };

      static int_T rt_LoggedOutputNumDimensions[] = {
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1
      };

      static int_T rt_LoggedOutputDimensions[] = {
        4,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        6,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        3,
        7,
        4,
        1,
        2,
        1
      };

      static boolean_T rt_LoggedOutputIsVarDims[] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      };

      static void* rt_LoggedCurrentSignalDimensions[] = {
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL)
      };

      static int_T rt_LoggedCurrentSignalDimensionsSize[] = {
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4
      };

      static BuiltInDTypeId rt_LoggedOutputDataTypeIds[] = {
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_BOOLEAN,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_BOOLEAN
      };

      static int_T rt_LoggedOutputComplexSignals[] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      };

      static const char_T *rt_LoggedOutputLabels[] = {
        "",
        "<X>",
        "<Y>",
        "<Z>",
        "<yaw>",
        "<pitch>",
        "<roll>",
        "<dx>",
        "<dy>",
        "<dz>",
        "<p>",
        "<q>",
        "<r>",
        "",
        "",
        "<ddx>",
        "<ddy>",
        "<ddz>",
        "<p>",
        "<q>",
        "<r>",
        "<altitude_sonar>",
        "<prs>",
        "",
        "",
        "",
        "",
        "",
        "" };

      static const char_T *rt_LoggedOutputBlockNames[] = {
        "Drone_Compensator/motors_refout",
        "Drone_Compensator/X",
        "Drone_Compensator/Y",
        "Drone_Compensator/Z",
        "Drone_Compensator/yaw",
        "Drone_Compensator/pitch",
        "Drone_Compensator/roll",
        "Drone_Compensator/dx",
        "Drone_Compensator/dy",
        "Drone_Compensator/dz",
        "Drone_Compensator/pb",
        "Drone_Compensator/qb",
        "Drone_Compensator/rb",
        "Drone_Compensator/controlModePosVSOrient_flagout",
        "Drone_Compensator/pose_refout",
        "Drone_Compensator/ddxb",
        "Drone_Compensator/ddyb",
        "Drone_Compensator/ddzb",
        "Drone_Compensator/pa",
        "Drone_Compensator/qa",
        "Drone_Compensator/ra",
        "Drone_Compensator/altitude_sonarb",
        "Drone_Compensator/prsb",
        "Drone_Compensator/opticalFlow_datout",
        "Drone_Compensator/sensordataCalib_datout",
        "Drone_Compensator/posVIS_datout",
        "Drone_Compensator/usePosVIS_flagout",
        "Drone_Compensator/batteryStatus_datout",
        "Drone_Compensator/takeoff_flagout" };

      static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert[] = {
        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_BOOLEAN, SS_BOOLEAN, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_BOOLEAN, SS_BOOLEAN, 0, 0, 0, 1.0, 0, 0.0 }
      };

      static RTWLogSignalInfo rt_LoggedOutputSignalInfo[] = {
        {
          29,
          rt_LoggedOutputWidths,
          rt_LoggedOutputNumDimensions,
          rt_LoggedOutputDimensions,
          rt_LoggedOutputIsVarDims,
          rt_LoggedCurrentSignalDimensions,
          rt_LoggedCurrentSignalDimensionsSize,
          rt_LoggedOutputDataTypeIds,
          rt_LoggedOutputComplexSignals,
          (NULL),

          { rt_LoggedOutputLabels },
          (NULL),
          (NULL),
          (NULL),

          { rt_LoggedOutputBlockNames },

          { (NULL) },
          (NULL),
          rt_RTWLogDataTypeConvert
        }
      };

      rtliSetLogYSignalInfo(Drone_Compensator_M->rtwLogInfo,
                            rt_LoggedOutputSignalInfo);

      /* set currSigDims field */
      rt_LoggedCurrentSignalDimensions[0] = &rt_LoggedOutputWidths[0];
      rt_LoggedCurrentSignalDimensions[1] = &rt_LoggedOutputWidths[1];
      rt_LoggedCurrentSignalDimensions[2] = &rt_LoggedOutputWidths[2];
      rt_LoggedCurrentSignalDimensions[3] = &rt_LoggedOutputWidths[3];
      rt_LoggedCurrentSignalDimensions[4] = &rt_LoggedOutputWidths[4];
      rt_LoggedCurrentSignalDimensions[5] = &rt_LoggedOutputWidths[5];
      rt_LoggedCurrentSignalDimensions[6] = &rt_LoggedOutputWidths[6];
      rt_LoggedCurrentSignalDimensions[7] = &rt_LoggedOutputWidths[7];
      rt_LoggedCurrentSignalDimensions[8] = &rt_LoggedOutputWidths[8];
      rt_LoggedCurrentSignalDimensions[9] = &rt_LoggedOutputWidths[9];
      rt_LoggedCurrentSignalDimensions[10] = &rt_LoggedOutputWidths[10];
      rt_LoggedCurrentSignalDimensions[11] = &rt_LoggedOutputWidths[11];
      rt_LoggedCurrentSignalDimensions[12] = &rt_LoggedOutputWidths[12];
      rt_LoggedCurrentSignalDimensions[13] = &rt_LoggedOutputWidths[13];
      rt_LoggedCurrentSignalDimensions[14] = &rt_LoggedOutputWidths[14];
      rt_LoggedCurrentSignalDimensions[15] = &rt_LoggedOutputWidths[15];
      rt_LoggedCurrentSignalDimensions[16] = &rt_LoggedOutputWidths[16];
      rt_LoggedCurrentSignalDimensions[17] = &rt_LoggedOutputWidths[17];
      rt_LoggedCurrentSignalDimensions[18] = &rt_LoggedOutputWidths[18];
      rt_LoggedCurrentSignalDimensions[19] = &rt_LoggedOutputWidths[19];
      rt_LoggedCurrentSignalDimensions[20] = &rt_LoggedOutputWidths[20];
      rt_LoggedCurrentSignalDimensions[21] = &rt_LoggedOutputWidths[21];
      rt_LoggedCurrentSignalDimensions[22] = &rt_LoggedOutputWidths[22];
      rt_LoggedCurrentSignalDimensions[23] = &rt_LoggedOutputWidths[23];
      rt_LoggedCurrentSignalDimensions[24] = &rt_LoggedOutputWidths[24];
      rt_LoggedCurrentSignalDimensions[25] = &rt_LoggedOutputWidths[25];
      rt_LoggedCurrentSignalDimensions[26] = &rt_LoggedOutputWidths[26];
      rt_LoggedCurrentSignalDimensions[27] = &rt_LoggedOutputWidths[27];
      rt_LoggedCurrentSignalDimensions[28] = &rt_LoggedOutputWidths[28];
    }

    rtliSetLogY(Drone_Compensator_M->rtwLogInfo, "yout");
  }

  /* block I/O */
  (void) memset(((void *) Drone_Compensator_B), 0,
                sizeof(B_Drone_Compensator_T));

  /* states (dwork) */
  (void) memset((void *)Drone_Compensator_DW, 0,
                sizeof(DW_Drone_Compensator_T));

  /* external inputs */
  (*Drone_Compensator_U_controlModePosVSOrient_flagin) = false;
  (void) memset(Drone_Compensator_U_pos_refin, 0,
                3U*sizeof(real_T));
  (*Drone_Compensator_U_takeoff_flag) = false;
  (void) memset(Drone_Compensator_U_orient_refin, 0,
                3U*sizeof(real_T));
  (*Drone_Compensator_U_ddx) = 0.0;
  (*Drone_Compensator_U_ddy) = 0.0;
  (*Drone_Compensator_U_ddz) = 0.0;
  (*Drone_Compensator_U_p) = 0.0;
  (*Drone_Compensator_U_q) = 0.0;
  (*Drone_Compensator_U_r) = 0.0;
  (*Drone_Compensator_U_altitude_sonar) = 0.0;
  (*Drone_Compensator_U_prs) = 0.0;
  (void) memset(Drone_Compensator_U_opticalFlow_datin, 0,
                3U*sizeof(real_T));
  (void) memset(Drone_Compensator_U_sensordataCalib_datin, 0,
                7U*sizeof(real_T));
  (void) memset(Drone_Compensator_U_posVIS_datin, 0,
                4U*sizeof(real_T));
  (*Drone_Compensator_U_usePosVIS_flagin) = 0.0;
  (void) memset(Drone_Compensator_U_batteryStatus_datin, 0,
                2U*sizeof(real_T));

  /* external outputs */
  (void) memset(&Drone_Compensator_Y_motors_refout[0], 0,
                4U*sizeof(real_T));
  (*Drone_Compensator_Y_X) = 0.0;
  (*Drone_Compensator_Y_Y) = 0.0;
  (*Drone_Compensator_Y_Z) = 0.0;
  (*Drone_Compensator_Y_yaw) = 0.0;
  (*Drone_Compensator_Y_pitch) = 0.0;
  (*Drone_Compensator_Y_roll) = 0.0;
  (*Drone_Compensator_Y_dx) = 0.0;
  (*Drone_Compensator_Y_dy) = 0.0;
  (*Drone_Compensator_Y_dz) = 0.0;
  (*Drone_Compensator_Y_pb) = 0.0;
  (*Drone_Compensator_Y_qb) = 0.0;
  (*Drone_Compensator_Y_rb) = 0.0;
  (*Drone_Compensator_Y_controlModePosVSOrient_flagout) = false;
  (void) memset(&Drone_Compensator_Y_pose_refout[0], 0,
                6U*sizeof(real_T));
  (*Drone_Compensator_Y_ddxb) = 0.0;
  (*Drone_Compensator_Y_ddyb) = 0.0;
  (*Drone_Compensator_Y_ddzb) = 0.0;
  (*Drone_Compensator_Y_pa) = 0.0;
  (*Drone_Compensator_Y_qa) = 0.0;
  (*Drone_Compensator_Y_ra) = 0.0;
  (*Drone_Compensator_Y_altitude_sonarb) = 0.0;
  (*Drone_Compensator_Y_prsb) = 0.0;
  (void) memset(&Drone_Compensator_Y_opticalFlow_datout[0], 0,
                3U*sizeof(real_T));
  (void) memset(&Drone_Compensator_Y_sensordataCalib_datout[0], 0,
                7U*sizeof(real_T));
  (void) memset(&Drone_Compensator_Y_posVIS_datout[0], 0,
                4U*sizeof(real_T));
  (*Drone_Compensator_Y_usePosVIS_flagout) = 0.0;
  (void) memset(&Drone_Compensator_Y_batteryStatus_datout[0], 0,
                2U*sizeof(real_T));
  (*Drone_Compensator_Y_takeoff_flagout) = false;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Drone_Compensator_M->rtwLogInfo, 0.0,
    rtmGetTFinal(Drone_Compensator_M), Drone_Compensator_M->Timing.stepSize0,
    (&rtmGetErrorStatus(Drone_Compensator_M)));

  /* InitializeConditions for Atomic SubSystem: '<Root>/Drone_Compensator' */
  Drone_Co_Drone_Compensator_Init(&Drone_Compensator_DW->Drone_Compensator_d,
    (P_Drone_Compensator_Drone_Com_T *)&Drone_Compensator_P->Drone_Compensator_d);

  /* End of InitializeConditions for SubSystem: '<Root>/Drone_Compensator' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
