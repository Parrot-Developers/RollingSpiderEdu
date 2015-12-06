/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Drone_Compensator.c
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

#include "Drone_Compensator.h"
#include "Drone_Compensator_private.h"

/* Output and update for atomic system: '<S1>/ControllerPolePlace' */
void Drone_Compe_ControllerPolePlace(const real_T rtu_pos_refin[3], boolean_T
  rtu_takeoff_flag, const real_T rtu_orient_refin[3], boolean_T
  rtu_controlModePosVSOrient_flag, const real_T rtu_states_estimin[2], const
  real_T rtu_states_estimin_m[2], real_T rtu_states_estimin_l, const real_T
  rtu_states_estimin_m4[3], real_T rtu_states_estimin_ly, const real_T
  rtu_states_estimin_le[3], B_ControllerPolePlace_Drone_C_T *localB,
  P_ControllerPolePlace_Drone_C_T *localP, P_Drone_Compensator_T
  *Drone_Compensator_P)
{
  /* local block i/o variables */
  real_T rtb_SaturationThrust;
  real_T y;
  real_T y_0;
  real_T rtb_TmpSignalConversionAtProduc[4];
  real_T rtu_states_estimin_0[12];
  real_T tmp[48];
  real_T rtu_states_estimin_1[12];
  int32_T i;
  int32_T i_0;
  real_T tmp_0[4];
  real_T u1;
  real_T u2;
  real_T tmp_1;

  /* Switch: '<S6>/PosVSOrient_Switch' incorporates:
   *  Constant: '<S6>/dz_ref'
   *  Constant: '<S6>/velocitiesPos_ref'
   *  Constant: '<S6>/velocitiesRot_ref'
   */
  if (rtu_controlModePosVSOrient_flag) {
    localB->PosVSOrient_Switch[0] = rtu_pos_refin[0];
    localB->PosVSOrient_Switch[1] = rtu_pos_refin[1];
  } else {
    localB->PosVSOrient_Switch[0] = rtu_states_estimin[0];
    localB->PosVSOrient_Switch[1] = rtu_states_estimin[1];
  }

  localB->PosVSOrient_Switch[2] = rtu_pos_refin[2];
  localB->PosVSOrient_Switch[3] = rtu_orient_refin[0];
  localB->PosVSOrient_Switch[4] = rtu_orient_refin[1];
  localB->PosVSOrient_Switch[5] = rtu_orient_refin[2];
  if (rtu_controlModePosVSOrient_flag) {
    localB->PosVSOrient_Switch[6] = localP->velocitiesPos_ref_Value[0];
    localB->PosVSOrient_Switch[7] = localP->velocitiesPos_ref_Value[1];
    localB->PosVSOrient_Switch[8] = localP->velocitiesPos_ref_Value[2];
  } else {
    localB->PosVSOrient_Switch[6] = rtu_states_estimin_m[0];
    localB->PosVSOrient_Switch[7] = rtu_states_estimin_m[1];
    localB->PosVSOrient_Switch[8] = localP->dz_ref_Value;
  }

  localB->PosVSOrient_Switch[9] = localP->velocitiesRot_ref_Value[0];
  localB->PosVSOrient_Switch[10] = localP->velocitiesRot_ref_Value[1];
  localB->PosVSOrient_Switch[11] = localP->velocitiesRot_ref_Value[2];

  /* End of Switch: '<S6>/PosVSOrient_Switch' */

  /* Sum: '<S2>/Add' */
  rtu_states_estimin_0[0] = rtu_states_estimin[0];
  rtu_states_estimin_0[1] = rtu_states_estimin[1];
  rtu_states_estimin_0[2] = rtu_states_estimin_l;
  rtu_states_estimin_0[3] = rtu_states_estimin_m4[0];
  rtu_states_estimin_0[4] = rtu_states_estimin_m4[1];
  rtu_states_estimin_0[5] = rtu_states_estimin_m4[2];
  rtu_states_estimin_0[6] = rtu_states_estimin_m[0];
  rtu_states_estimin_0[7] = rtu_states_estimin_m[1];
  rtu_states_estimin_0[8] = rtu_states_estimin_ly;
  rtu_states_estimin_0[9] = rtu_states_estimin_le[0];
  rtu_states_estimin_0[10] = rtu_states_estimin_le[1];
  rtu_states_estimin_0[11] = rtu_states_estimin_le[2];

  /* Gain: '<S4>/Gain' */
  for (i = 0; i < 12; i++) {
    tmp[i << 2] = -Drone_Compensator_P->K_poleplace[i << 2];
    tmp[1 + (i << 2)] = -Drone_Compensator_P->K_poleplace[(i << 2) + 1];
    tmp[2 + (i << 2)] = -Drone_Compensator_P->K_poleplace[(i << 2) + 2];
    tmp[3 + (i << 2)] = -Drone_Compensator_P->K_poleplace[(i << 2) + 3];
  }

  /* Sum: '<S2>/Add' incorporates:
   *  Gain: '<S4>/Gain'
   */
  for (i = 0; i < 12; i++) {
    rtu_states_estimin_1[i] = rtu_states_estimin_0[i] -
      localB->PosVSOrient_Switch[i];
  }

  /* Gain: '<S4>/Gain' */
  for (i = 0; i < 4; i++) {
    rtb_TmpSignalConversionAtProduc[i] = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      rtb_TmpSignalConversionAtProduc[i] += tmp[(i_0 << 2) + i] *
        rtu_states_estimin_1[i_0];
    }
  }

  /* Switch: '<S8>/TakeoffOrControl_Switch' incorporates:
   *  Constant: '<S8>/HoverThrustLinearizationPoint'
   *  Gain: '<S8>/takeoff_Gain'
   */
  if (rtu_takeoff_flag) {
    rtb_SaturationThrust = -Drone_Compensator_P->quad.g *
      Drone_Compensator_P->quad.M *
      Drone_Compensator_P->controlHelperParams.takeoff_gain;
  } else {
    rtb_SaturationThrust = rtb_TmpSignalConversionAtProduc[0];
  }

  /* End of Switch: '<S8>/TakeoffOrControl_Switch' */

  /* Sum: '<S8>/Add1' incorporates:
   *  Constant: '<S8>/HoverThrustLinearizationPoint'
   */
  rtb_SaturationThrust += -Drone_Compensator_P->quad.g *
    Drone_Compensator_P->quad.M;

  /* Saturate: '<S8>/SaturationThrust' */
  u1 = -Drone_Compensator_P->controlHelperParams.totalThrust_maxRelative *
    Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max * 4.0;
  u2 = Drone_Compensator_P->controlHelperParams.totalThrust_maxRelative *
    Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max * 4.0;
  if (rtb_SaturationThrust > u2) {
    rtb_SaturationThrust = u2;
  } else {
    if (rtb_SaturationThrust < u1) {
      rtb_SaturationThrust = u1;
    }
  }

  /* End of Saturate: '<S8>/SaturationThrust' */

  /* SignalConversion: '<S7>/TmpSignal ConversionAtProductInport2' */
  u1 = rtb_SaturationThrust;

  /* Saturate: '<S7>/Saturation2' */
  u2 = -0.05 * Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;

  /* Gain: '<S9>/ThrustToW2_Gain' */
  y = 1.0 / Drone_Compensator_P->quadEDT.w2ToThrust_gain;

  /* Gain: '<S9>/W2ToMotorsCmd_Gain' */
  y_0 = 1.0 / Drone_Compensator_P->quadEDT.motorcommandToW2_gain;

  /* Product: '<S7>/Product' incorporates:
   *  Constant: '<S7>/TorquetotalThrustToThrustperMotor'
   *  Saturate: '<S7>/Saturation2'
   *  SignalConversion: '<S7>/TmpSignal ConversionAtProductInport2'
   */
  for (i = 0; i < 4; i++) {
    tmp_1 = Drone_Compensator_P->controlHelperParams.Q2Ts[i + 12] *
      rtb_TmpSignalConversionAtProduc[3] +
      (Drone_Compensator_P->controlHelperParams.Q2Ts[i + 8] *
       rtb_TmpSignalConversionAtProduc[2] +
       (Drone_Compensator_P->controlHelperParams.Q2Ts[i + 4] *
        rtb_TmpSignalConversionAtProduc[1] +
        Drone_Compensator_P->controlHelperParams.Q2Ts[i] * u1));
    tmp_0[i] = tmp_1;
  }

  /* End of Product: '<S7>/Product' */

  /* Gain: '<S9>/W2ToMotorsCmd_Gain' incorporates:
   *  Gain: '<S9>/MotorsRotationDirection'
   *  Gain: '<S9>/ThrustToW2_Gain'
   *  Saturate: '<S7>/Saturation2'
   */
  if (tmp_0[0] > u2) {
    tmp_1 = u2;
  } else if (tmp_0[0] <
             -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max)
  {
    tmp_1 = -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;
  } else {
    tmp_1 = tmp_0[0];
  }

  localB->W2ToMotorsCmd_Gain[0] = y * tmp_1 *
    localP->MotorsRotationDirection_Gain[0] * y_0;
  if (tmp_0[1] > u2) {
    tmp_1 = u2;
  } else if (tmp_0[1] <
             -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max)
  {
    tmp_1 = -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;
  } else {
    tmp_1 = tmp_0[1];
  }

  localB->W2ToMotorsCmd_Gain[1] = y * tmp_1 *
    localP->MotorsRotationDirection_Gain[1] * y_0;
  if (tmp_0[2] > u2) {
    tmp_1 = u2;
  } else if (tmp_0[2] <
             -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max)
  {
    tmp_1 = -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;
  } else {
    tmp_1 = tmp_0[2];
  }

  localB->W2ToMotorsCmd_Gain[2] = y * tmp_1 *
    localP->MotorsRotationDirection_Gain[2] * y_0;
  if (!(tmp_0[3] > u2)) {
    if (tmp_0[3] <
        -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max) {
      u2 = -Drone_Compensator_P->controlHelperParams.motorsThrustperMotor_max;
    } else {
      u2 = tmp_0[3];
    }
  }

  localB->W2ToMotorsCmd_Gain[3] = y * u2 * localP->MotorsRotationDirection_Gain
    [3] * y_0;
}

/*
 * Output and update for enable system:
 *    '<S95>/MeasurementUpdate'
 *    '<S155>/MeasurementUpdate'
 */
void Drone_Compens_MeasurementUpdate(boolean_T rtu_Enable, const real_T rtu_Lk[4],
  const real_T rtu_yk[2], const real_T rtu_yhatkk1[2],
  B_MeasurementUpdate_Drone_Com_T *localB)
{
  real_T rtu_yk_idx_0;
  real_T rtu_yk_idx_1;

  /* Outputs for Enabled SubSystem: '<S95>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S120>/Enable'
   */
  if (rtu_Enable) {
    /* Sum: '<S120>/Sum' incorporates:
     *  Product: '<S120>/Product3'
     */
    rtu_yk_idx_0 = rtu_yk[0] - rtu_yhatkk1[0];
    rtu_yk_idx_1 = rtu_yk[1] - rtu_yhatkk1[1];

    /* Product: '<S120>/Product3' */
    localB->Product3[0] = 0.0;
    localB->Product3[0] += rtu_Lk[0] * rtu_yk_idx_0;
    localB->Product3[0] += rtu_Lk[2] * rtu_yk_idx_1;
    localB->Product3[1] = 0.0;
    localB->Product3[1] += rtu_Lk[1] * rtu_yk_idx_0;
    localB->Product3[1] += rtu_Lk[3] * rtu_yk_idx_1;
  }

  /* End of Outputs for SubSystem: '<S95>/MeasurementUpdate' */
}

/*
 * Output and update for atomic system:
 *    '<S71>/UseCurrentEstimator'
 *    '<S134>/UseCurrentEstimator'
 */
void Drone_Compe_UseCurrentEstimator(boolean_T rtu_Enablek, const real_T rtu_Mk
  [4], const real_T rtu_uk[2], const real_T rtu_yk[2], const real_T rtu_Ck[4],
  const real_T rtu_Dk[4], const real_T rtu_xhatkk1[2],
  B_UseCurrentEstimator_Drone_C_T *localB)
{
  real_T rtu_yk_idx_0;
  real_T rtu_yk_idx_1;

  /* Outputs for Enabled SubSystem: '<S100>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S121>/Enable'
   */
  if (rtu_Enablek) {
    /* Sum: '<S121>/Add1' incorporates:
     *  Product: '<S121>/Product'
     *  Product: '<S121>/Product1'
     */
    rtu_yk_idx_0 = (rtu_yk[0] - (rtu_Ck[0] * rtu_xhatkk1[0] + rtu_Ck[2] *
      rtu_xhatkk1[1])) - (rtu_Dk[0] * rtu_uk[0] + rtu_Dk[2] * rtu_uk[1]);
    rtu_yk_idx_1 = (rtu_yk[1] - (rtu_Ck[1] * rtu_xhatkk1[0] + rtu_Ck[3] *
      rtu_xhatkk1[1])) - (rtu_Dk[1] * rtu_uk[0] + rtu_Dk[3] * rtu_uk[1]);

    /* Product: '<S121>/Product2' */
    localB->Product2[0] = 0.0;
    localB->Product2[0] += rtu_Mk[0] * rtu_yk_idx_0;
    localB->Product2[0] += rtu_Mk[2] * rtu_yk_idx_1;
    localB->Product2[1] = 0.0;
    localB->Product2[1] += rtu_Mk[1] * rtu_yk_idx_0;
    localB->Product2[1] += rtu_Mk[3] * rtu_yk_idx_1;
  }

  /* End of Outputs for SubSystem: '<S100>/Enabled Subsystem' */

  /* Sum: '<S100>/Add' */
  localB->Add[0] = localB->Product2[0] + rtu_xhatkk1[0];
  localB->Add[1] = localB->Product2[1] + rtu_xhatkk1[1];
}

/* Initial conditions for atomic system: '<Root>/Drone_Compensator' */
void Drone_Co_Drone_Compensator_Init(DW_Drone_Compensator_Drone_Co_T *localDW,
  P_Drone_Compensator_Drone_Com_T *localP)
{
  int32_T i;

  /* InitializeConditions for DiscreteFir: '<S13>/FIR_IMUaccel' */
  localDW->FIR_IMUaccel_circBuf = 0;
  for (i = 0; i < 15; i++) {
    localDW->FIR_IMUaccel_states[i] = localP->FIR_IMUaccel_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFir: '<S13>/FIR_IMUaccel' */

  /* InitializeConditions for DiscreteFilter: '<S13>/IIR_IMUgyro_r' */
  for (i = 0; i < 5; i++) {
    localDW->IIR_IMUgyro_r_states[i] = localP->IIR_IMUgyro_r_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFilter: '<S13>/IIR_IMUgyro_r' */

  /* InitializeConditions for MATLAB Function: '<S3>/EstimatorOrientation' */
  localDW->yaw_cur = 0.0;
  localDW->pitch_cur = 0.0;
  localDW->roll_cur = 0.0;

  /* InitializeConditions for Delay: '<S69>/Delay' */
  localDW->Delay_DSTATE[0] = localP->Delay_InitialCondition;
  localDW->Delay_DSTATE[1] = localP->Delay_InitialCondition;

  /* InitializeConditions for DiscreteFilter: '<S72>/IIRgyroz' */
  for (i = 0; i < 10; i++) {
    localDW->IIRgyroz_states[i] = localP->IIRgyroz_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFilter: '<S72>/IIRgyroz' */

  /* InitializeConditions for UnitDelay: '<S122>/UD' */
  localDW->UD_DSTATE[0] = localP->DiscreteDerivative_ICPrevScaled;
  localDW->UD_DSTATE[1] = localP->DiscreteDerivative_ICPrevScaled;

  /* InitializeConditions for Delay: '<S68>/Delay' */
  localDW->Delay_DSTATE_b[0] = localP->Delay_InitialCondition_h;
  localDW->Delay_DSTATE_b[1] = localP->Delay_InitialCondition_h;

  /* InitializeConditions for Delay: '<S10>/Delay2' */
  localDW->Delay2_DSTATE = localP->Delay2_InitialCondition;
  for (i = 0; i < 5; i++) {
    /* InitializeConditions for DiscreteFilter: '<S15>/pressureFilter_IIR' */
    localDW->pressureFilter_IIR_states[i] =
      localP->pressureFilter_IIR_InitialState;

    /* InitializeConditions for DiscreteFilter: '<S15>/soonarFilter_IIR' */
    localDW->soonarFilter_IIR_states[i] = localP->soonarFilter_IIR_InitialStates;
  }

  /* InitializeConditions for Delay: '<S14>/MemoryX' */
  localDW->icLoad = 1U;

  /* InitializeConditions for Delay: '<S71>/MemoryX' */
  localDW->icLoad_l = 1U;

  /* InitializeConditions for Delay: '<S3>/Delay1' */
  localDW->Delay1_DSTATE[0] = localP->Delay1_InitialCondition;
  localDW->Delay1_DSTATE[1] = localP->Delay1_InitialCondition;

  /* InitializeConditions for Delay: '<S134>/MemoryX' */
  localDW->icLoad_j = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S69>/SimplyIntegrateVelocity' */
  localDW->SimplyIntegrateVelocity_DSTATE[0] =
    localP->SimplyIntegrateVelocity_IC;
  localDW->SimplyIntegrateVelocity_DSTATE[1] =
    localP->SimplyIntegrateVelocity_IC;
  localDW->SimplyIntegrateVelocity_PrevRes = 2;
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

  /* Gain: '<S13>/inverseIMU_gain' incorporates:
   *  Bias: '<S13>/Assuming that calib was done level!'
   *  Sum: '<S13>/Sum1'
   */
  for (i = 0; i < 6; i++) {
    inverseIMU_gain[i] = (localB->sensordata_datin[i] -
                          (localB->sensordataCalib_datin[i] +
      localP->Assumingthatcalibwasdonelevel_B[i])) *
      Drone_Compensator_P->quadEDT.inverseIMU_gain[i];
  }

  /* End of Gain: '<S13>/inverseIMU_gain' */

  /* DiscreteFir: '<S13>/FIR_IMUaccel' */
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

  /* End of DiscreteFir: '<S13>/FIR_IMUaccel' */

  /* DiscreteFilter: '<S13>/IIR_IMUgyro_r' */
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

  /* End of DiscreteFilter: '<S13>/IIR_IMUgyro_r' */

  /* SignalConversion: '<S11>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  MATLAB Function: '<S3>/EstimatorOrientation'
   */
  rtb_TmpSignalConversionAtSFun_0 = rtb_r;

  /* MATLAB Function: '<S3>/EstimatorOrientation' incorporates:
   *  Constant: '<S187>/Constant'
   *  Constant: '<S3>/sampleTime'
   *  Constant: '<S3>/sampleTime1'
   *  Logic: '<S13>/Logical Operator'
   *  RelationalOperator: '<S187>/Compare'
   *  SignalConversion: '<S11>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorOrientation': '<S11>:1' */
  /* '<S11>:1:4' */
  /* '<S11>:1:5' */
  /* '<S11>:1:6' */
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

  /* '<S11>:1:18' */
  /* '<S11>:1:20' */
  localDW->yaw_cur = pressureFilter_IIR_tmp;

  /* '<S11>:1:21' */
  localDW->pitch_cur = rtb_Add1_l;

  /* '<S11>:1:22' */
  localDW->roll_cur = rtb_r;

  /* '<S11>:1:24' */
  localB->orient_estimout[0] = pressureFilter_IIR_tmp;
  localB->orient_estimout[1] = rtb_Add1_l;
  localB->orient_estimout[2] = rtb_r;

  /* '<S11>:1:25' */
  localB->dorient_estimout[0] = inverseIMU_gain[3];
  localB->dorient_estimout[1] = inverseIMU_gain[4];
  localB->dorient_estimout[2] = rtb_TmpSignalConversionAtSFun_0;

  /* Abs: '<S135>/Abs' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[1]);

  /* RelationalOperator: '<S184>/Compare' incorporates:
   *  Constant: '<S184>/Constant'
   */
  rtb_Compare_ii = (rtb_Dk1uk1_p <= localP->maxp3_const);

  /* Abs: '<S135>/Abs1' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[2]);

  /* MATLAB Function: '<S135>/abs' incorporates:
   *  Delay: '<S69>/Delay'
   *  Sum: '<S135>/Add1'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs': '<S182>:1' */
  /* '<S182>:1:2' */
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

  /* End of MATLAB Function: '<S135>/abs' */

  /* Logic: '<S135>/Logical Operator3' incorporates:
   *  Constant: '<S183>/Constant'
   *  Constant: '<S185>/Constant'
   *  Constant: '<S186>/Constant'
   *  RelationalOperator: '<S183>/Compare'
   *  RelationalOperator: '<S185>/Compare'
   *  RelationalOperator: '<S186>/Compare'
   */
  rtb_LogicalOperator3 = ((localB->posVIS_datin[0] !=
    localP->checkifPosavailable_const) && rtb_Compare_ii && (rtb_Dk1uk1_p <=
    localP->maxq3_const) && (rtb_r < localP->planarjumpsVISPOS_const));

  /* Abs: '<S72>/Abs2' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[1]);

  /* RelationalOperator: '<S125>/Compare' incorporates:
   *  Constant: '<S125>/Constant'
   */
  rtb_Compare_ii = (rtb_Dk1uk1_p <= localP->maxp_const);

  /* Abs: '<S72>/Abs3' */
  rtb_Dk1uk1_p = fabs(localB->orient_estimout[2]);

  /* RelationalOperator: '<S127>/Compare' incorporates:
   *  Constant: '<S127>/Constant'
   */
  rtb_Compare_du = (rtb_Dk1uk1_p <= localP->maxq_const);

  /* Abs: '<S72>/Abs' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[3]);

  /* RelationalOperator: '<S129>/Compare' incorporates:
   *  Constant: '<S129>/Constant'
   */
  rtb_Compare_po = (rtb_Dk1uk1_p <= localP->maxw1_const);

  /* Abs: '<S72>/Abs1' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[4]);

  /* RelationalOperator: '<S130>/Compare' incorporates:
   *  Constant: '<S130>/Constant'
   */
  rtb_Compare_cz = (rtb_Dk1uk1_p <= localP->maxw2_const);

  /* DiscreteFilter: '<S72>/IIRgyroz' */
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

  /* End of DiscreteFilter: '<S72>/IIRgyroz' */

  /* SampleTimeMath: '<S122>/TSamp'
   *
   * About '<S122>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  absxk = rtb_Dk1uk1[0] * localP->TSamp_WtEt;
  t = rtb_Dk1uk1[1] * localP->TSamp_WtEt;

  /* Abs: '<S72>/Abs6' incorporates:
   *  Sum: '<S122>/Diff'
   *  UnitDelay: '<S122>/UD'
   */
  rtb_Dk1uk1_p = fabs(absxk - localDW->UD_DSTATE[0]);

  /* RelationalOperator: '<S123>/Compare' incorporates:
   *  Constant: '<S123>/Constant'
   */
  rtb_Compare_l = (rtb_Dk1uk1_p <= localP->maxdw1_const);

  /* Abs: '<S72>/Abs7' incorporates:
   *  Sum: '<S122>/Diff'
   *  UnitDelay: '<S122>/UD'
   */
  rtb_Dk1uk1_p = fabs(t - localDW->UD_DSTATE[1]);

  /* Logic: '<S72>/Logical Operator' incorporates:
   *  Constant: '<S124>/Constant'
   *  RelationalOperator: '<S124>/Compare'
   */
  rtb_LogicalOperator_o = (rtb_Compare_ii && rtb_Compare_du && rtb_Compare_po &&
    rtb_Compare_cz && rtb_Compare_l && (rtb_Dk1uk1_p <= localP->maxdw2_const));

  /* Abs: '<S72>/Abs4' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[3]);

  /* RelationalOperator: '<S126>/Compare' incorporates:
   *  Constant: '<S126>/Constant'
   */
  rtb_Compare_ii = (rtb_Dk1uk1_p <= localP->maxp2_const);

  /* Abs: '<S72>/Abs5' */
  rtb_Dk1uk1_p = fabs(inverseIMU_gain[4]);

  /* Logic: '<S72>/Logical Operator1' incorporates:
   *  Constant: '<S128>/Constant'
   *  RelationalOperator: '<S128>/Compare'
   */
  rtb_Compare_po = (rtb_Compare_ii && (rtb_Dk1uk1_p <= localP->maxq2_const));

  /* Gain: '<S68>/opticalFlowToVelocity_gain' */
  rtb_opticalFlowToVelocity_gain[0] = localP->opticalFlowToVelocity_gain_Gain *
    localB->opticalFlow_datin[0];
  rtb_opticalFlowToVelocity_gain[1] = localP->opticalFlowToVelocity_gain_Gain *
    localB->opticalFlow_datin[1];
  rtb_opticalFlowToVelocity_gain[2] = localP->opticalFlowToVelocity_gain_Gain *
    localB->opticalFlow_datin[2];

  /* Abs: '<S72>/Abs8' incorporates:
   *  Delay: '<S68>/Delay'
   *  Sum: '<S72>/Add'
   */
  rtb_Dk1uk1_p = fabs(rtb_opticalFlowToVelocity_gain[0] -
                      localDW->Delay_DSTATE_b[0]);

  /* RelationalOperator: '<S131>/Compare' incorporates:
   *  Constant: '<S131>/Constant'
   */
  rtb_Compare_cz = (rtb_Dk1uk1_p <= localP->maxw3_const);

  /* Abs: '<S72>/Abs9' incorporates:
   *  Delay: '<S68>/Delay'
   *  Sum: '<S72>/Add'
   */
  rtb_Dk1uk1_p = fabs(rtb_opticalFlowToVelocity_gain[1] -
                      localDW->Delay_DSTATE_b[1]);

  /* RelationalOperator: '<S132>/Compare' incorporates:
   *  Constant: '<S132>/Constant'
   */
  rtb_Compare_l = (rtb_Dk1uk1_p <= localP->maxw4_const);

  /* Gain: '<S10>/invertzaxisGain' */
  rtb_invertzaxisGain = localP->invertzaxisGain_Gain * localB->sensordata_datin
    [6];

  /* Delay: '<S10>/Delay2' */
  rtb_Dk1uk1_p = localDW->Delay2_DSTATE;

  /* Saturate: '<S15>/SaturationSonar' */
  if (rtb_invertzaxisGain > -Drone_Compensator_P->quadEDT.altSenor_min) {
    rtb_TmpSignalConversionAtSFun_0 = -Drone_Compensator_P->quadEDT.altSenor_min;
  } else if (rtb_invertzaxisGain < localP->SaturationSonar_LowerSat) {
    rtb_TmpSignalConversionAtSFun_0 = localP->SaturationSonar_LowerSat;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = rtb_invertzaxisGain;
  }

  /* Sum: '<S15>/Add' incorporates:
   *  Saturate: '<S15>/SaturationSonar'
   */
  rtb_Ckxhatkk1_e = rtb_Dk1uk1_p - rtb_TmpSignalConversionAtSFun_0;

  /* Abs: '<S15>/Absestdiff' */
  rtb_Ckxhatkk1_e = fabs(rtb_Ckxhatkk1_e);

  /* RelationalOperator: '<S67>/Compare' incorporates:
   *  Constant: '<S67>/Constant'
   */
  rtb_Compare_ii = (rtb_Ckxhatkk1_e <= localP->outlierJump_const);

  /* Gain: '<S10>/prsToAlt_gain' incorporates:
   *  Sum: '<S13>/Sum2'
   */
  rtb_altfrompress = 1.0 / Drone_Compensator_P->quadEDT.altToPrs_gain *
    (localB->sensordata_datin[7] - localB->sensordataCalib_datin[6]);

  /* DiscreteFilter: '<S15>/pressureFilter_IIR' */
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

  /* End of DiscreteFilter: '<S15>/pressureFilter_IIR' */

  /* Sum: '<S15>/Add1' */
  rtb_Ckxhatkk1_e -= rtb_Dk1uk1_p;

  /* Abs: '<S15>/Absestdiff1' */
  rtb_Ckxhatkk1_e = fabs(rtb_Ckxhatkk1_e);

  /* RelationalOperator: '<S65>/Compare' incorporates:
   *  Constant: '<S65>/Constant'
   */
  rtb_Compare_du = (rtb_Ckxhatkk1_e >= localP->currentEstimateVeryOffFromPress);

  /* DiscreteFilter: '<S15>/soonarFilter_IIR' */
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

  /* End of DiscreteFilter: '<S15>/soonarFilter_IIR' */

  /* Sum: '<S15>/Add2' */
  rtb_Ckxhatkk1_e -= rtb_Dk1uk1_p;

  /* Abs: '<S15>/Absestdiff2' */
  rtb_Ckxhatkk1_e = fabs(rtb_Ckxhatkk1_e);

  /* Logic: '<S15>/nicemeasurementor newupdateneeded' incorporates:
   *  Constant: '<S64>/Constant'
   *  Constant: '<S66>/Constant'
   *  Logic: '<S15>/findingoutliers'
   *  Logic: '<S15>/newupdateneeded'
   *  RelationalOperator: '<S64>/Compare'
   *  RelationalOperator: '<S66>/Compare'
   */
  rtb_nicemeasurementornewupdaten = ((rtb_Compare_ii && (rtb_invertzaxisGain <
    -Drone_Compensator_P->quadEDT.altSenor_min)) || (rtb_Compare_du &&
    (rtb_Ckxhatkk1_e >= localP->currentStateVeryOffsonarflt_con)));

  /* MATLAB Function: '<S10>/trafo_BodytoWorld_trans' */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorAltitude/trafo_BodytoWorld_trans': '<S17>:1' */
  /* '<S17>:1:2' */
  /* '<S17>:1:3' */
  /* '<S17>:1:4' */
  /* BBF > Inertial rotation matrix */
  /* '<S17>:1:6' */
  /* '<S17>:1:10' */
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

  /* Sum: '<S10>/Sum' incorporates:
   *  Constant: '<S10>/gravity'
   *  MATLAB Function: '<S10>/trafo_BodytoWorld_trans'
   */
  for (i = 0; i < 3; i++) {
    rtb_Sum[i] = ((tmp_1[i + 3] * localB->FIR_IMUaccel[1] + tmp_1[i] *
                   localB->FIR_IMUaccel[0]) + tmp_1[i + 6] *
                  localB->FIR_IMUaccel[2]) + localP->gravity_Value[i];
  }

  /* End of Sum: '<S10>/Sum' */

  /* Delay: '<S14>/MemoryX' incorporates:
   *  Constant: '<S14>/X0'
   *  Constant: '<S16>/Constant'
   *  RelationalOperator: '<S16>/Compare'
   */
  if (rtb_Dk1uk1_p > localP->outlierBelowFloor_const) {
    localDW->icLoad = 1U;
  }

  if (localDW->icLoad != 0) {
    localDW->MemoryX_DSTATE[0] = localP->X0_Value[0];
    localDW->MemoryX_DSTATE[1] = localP->X0_Value[1];
  }

  /* Outputs for Atomic SubSystem: '<S14>/UseCurrentEstimator' */
  /* Outputs for Enabled SubSystem: '<S42>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S63>/Enable'
   */
  if (rtb_nicemeasurementornewupdaten) {
    /* Sum: '<S63>/Add1' incorporates:
     *  Constant: '<S14>/C'
     *  Constant: '<S14>/D'
     *  Delay: '<S14>/MemoryX'
     *  Product: '<S63>/Product'
     *  Product: '<S63>/Product1'
     */
    rtb_Add1_l = (rtb_invertzaxisGain - (localP->C_Value[0] *
      localDW->MemoryX_DSTATE[0] + localP->C_Value[1] * localDW->MemoryX_DSTATE
      [1])) - localP->D_Value * rtb_Sum[2];

    /* Product: '<S63>/Product2' incorporates:
     *  Constant: '<S19>/KalmanGainM'
     */
    localB->Product2[0] = localP->KalmanGainM_Value_p[0] * rtb_Add1_l;
    localB->Product2[1] = localP->KalmanGainM_Value_p[1] * rtb_Add1_l;
  }

  /* End of Outputs for SubSystem: '<S42>/Enabled Subsystem' */

  /* Reshape: '<S14>/Reshapexhat' incorporates:
   *  Delay: '<S14>/MemoryX'
   *  Sum: '<S42>/Add'
   */
  localB->Reshapexhat[0] = localB->Product2[0] + localDW->MemoryX_DSTATE[0];
  localB->Reshapexhat[1] = localB->Product2[1] + localDW->MemoryX_DSTATE[1];

  /* End of Outputs for SubSystem: '<S14>/UseCurrentEstimator' */

  /* Logic: '<S72>/Logical Operator3' incorporates:
   *  Constant: '<S133>/Constant'
   *  Logic: '<S72>/Logical Operator2'
   *  RelationalOperator: '<S133>/Compare'
   */
  rtb_LogicalOperator3_j = ((rtb_LogicalOperator_o || rtb_Compare_po) &&
    rtb_Compare_cz && rtb_Compare_l && (localB->Reshapexhat[0] <=
    localP->minHeightforOF_const));

  /* Logic: '<S70>/Logical Operator' incorporates:
   *  Constant: '<S74>/Constant'
   *  Constant: '<S75>/Constant'
   *  RelationalOperator: '<S74>/Compare'
   *  RelationalOperator: '<S75>/Compare'
   */
  rtb_Compare_ii = ((rtb_opticalFlowToVelocity_gain[0] !=
                     localP->donotuseaccifopticalflowneverav) ||
                    (rtb_opticalFlowToVelocity_gain[1] !=
                     localP->donotuseaccifopticalflownever_g));

  /* RelationalOperator: '<S73>/Compare' incorporates:
   *  Constant: '<S73>/Constant'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/trafo_World2Body_trans': '<S76>:1' */
  /* '<S76>:1:3' */
  /* '<S76>:1:4' */
  /* '<S76>:1:5' */
  /* '<S76>:1:7' */
  /* '<S76>:1:11' */
  rtb_Compare_du = (localB->Reshapexhat[0] <=
                    localP->DeactivateAccelerationIfOFisnot);

  /* MATLAB Function: '<S70>/trafo_World2Body_trans' */
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

  /* Sum: '<S70>/Add' incorporates:
   *  Constant: '<S70>/gravity'
   *  Gain: '<S70>/gainaccinput'
   *  MATLAB Function: '<S70>/trafo_World2Body_trans'
   */
  for (i = 0; i < 3; i++) {
    rtb_vel_world[i] = localB->FIR_IMUaccel[i] - ((tmp_2[i + 3] *
      localP->gravity_Value_b[1] + tmp_2[i] * localP->gravity_Value_b[0]) +
      tmp_2[i + 6] * localP->gravity_Value_b[2]);
  }

  /* End of Sum: '<S70>/Add' */

  /* Product: '<S70>/Product' incorporates:
   *  Gain: '<S70>/gainaccinput'
   */
  rtb_Dk1uk1[0] = localP->gainaccinput_Gain * rtb_vel_world[0] * (real_T)
    rtb_Compare_ii * (real_T)rtb_Compare_du;
  rtb_Dk1uk1[1] = localP->gainaccinput_Gain * rtb_vel_world[1] * (real_T)
    rtb_Compare_ii * (real_T)rtb_Compare_du;

  /* Delay: '<S71>/MemoryX' incorporates:
   *  Constant: '<S71>/X0'
   */
  if (localDW->icLoad_l != 0) {
    localDW->MemoryX_DSTATE_g[0] = localP->X0_Value_j[0];
    localDW->MemoryX_DSTATE_g[1] = localP->X0_Value_j[1];
  }

  rtb_Add1[0] = localDW->MemoryX_DSTATE_g[0];
  rtb_Add1[1] = localDW->MemoryX_DSTATE_g[1];

  /* Outputs for Atomic SubSystem: '<S71>/UseCurrentEstimator' */

  /* Constant: '<S77>/KalmanGainM' incorporates:
   *  Constant: '<S71>/C'
   *  Constant: '<S71>/D'
   */
  Drone_Compe_UseCurrentEstimator(rtb_LogicalOperator3_j,
    localP->KalmanGainM_Value_n, rtb_Dk1uk1, &rtb_opticalFlowToVelocity_gain[0],
    localP->C_Value_j, localP->D_Value_b, rtb_Add1,
    &localB->UseCurrentEstimator_b);

  /* End of Outputs for SubSystem: '<S71>/UseCurrentEstimator' */

  /* Reshape: '<S71>/Reshapexhat' */
  localB->Reshapexhat_o[0] = localB->UseCurrentEstimator_b.Add[0];
  localB->Reshapexhat_o[1] = localB->UseCurrentEstimator_b.Add[1];

  /* SignalConversion: '<S18>/TmpSignal ConversionAt SFunction Inport2' incorporates:
   *  Delay: '<S3>/Delay1'
   *  MATLAB Function: '<S10>/trafo_WorldToBody_trans'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorAltitude/trafo_WorldToBody_trans': '<S18>:1' */
  /* '<S18>:1:2' */
  /* '<S18>:1:3' */
  /* '<S18>:1:4' */
  /* '<S18>:1:7' */
  /* '<S18>:1:11' */
  rtb_TmpSignalConversionAtSFun_0 = localDW->Delay1_DSTATE[0];
  scale = localDW->Delay1_DSTATE[1];

  /* MATLAB Function: '<S10>/trafo_WorldToBody_trans' incorporates:
   *  SignalConversion: '<S18>/TmpSignal ConversionAt SFunction Inport2'
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

  /* MATLAB Function: '<S69>/trafo_BodytoWorld_trans' incorporates:
   *  SignalConversion: '<S136>/TmpSignal ConversionAt SFunction Inport2'
   */
  /* MATLAB Function 'Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/trafo_BodytoWorld_trans': '<S136>:1' */
  /* '<S136>:1:2' */
  /* '<S136>:1:3' */
  /* '<S136>:1:4' */
  /* BBF > Inertial rotation matrix */
  /* '<S136>:1:6' */
  /* '<S136>:1:10' */
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

  /* End of MATLAB Function: '<S69>/trafo_BodytoWorld_trans' */

  /* Delay: '<S134>/MemoryX' incorporates:
   *  Constant: '<S134>/X0'
   */
  if (localDW->icLoad_j != 0) {
    localDW->MemoryX_DSTATE_m[0] = localP->X0_Value_k[0];
    localDW->MemoryX_DSTATE_m[1] = localP->X0_Value_k[1];
  }

  rtb_Add1[0] = localDW->MemoryX_DSTATE_m[0];
  rtb_Add1[1] = localDW->MemoryX_DSTATE_m[1];

  /* Outputs for Atomic SubSystem: '<S134>/UseCurrentEstimator' */

  /* Constant: '<S137>/KalmanGainM' incorporates:
   *  Constant: '<S134>/C'
   *  Constant: '<S134>/D'
   */
  Drone_Compe_UseCurrentEstimator(rtb_LogicalOperator3,
    localP->KalmanGainM_Value, &rtb_vel_world[0], &localB->posVIS_datin[0],
    localP->C_Value_f, localP->D_Value_o, rtb_Add1,
    &localB->UseCurrentEstimator_g);

  /* End of Outputs for SubSystem: '<S134>/UseCurrentEstimator' */

  /* DiscreteIntegrator: '<S69>/SimplyIntegrateVelocity' */
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

  /* Switch: '<S69>/UseIPPosSwitch' incorporates:
   *  DiscreteIntegrator: '<S69>/SimplyIntegrateVelocity'
   */
  if (localB->usePosVIS_flagin > localP->UseIPPosSwitch_Threshold) {
    localB->UseIPPosSwitch[0] = localB->UseCurrentEstimator_g.Add[0];
    localB->UseIPPosSwitch[1] = localB->UseCurrentEstimator_g.Add[1];
  } else {
    localB->UseIPPosSwitch[0] = localDW->SimplyIntegrateVelocity_DSTATE[0];
    localB->UseIPPosSwitch[1] = localDW->SimplyIntegrateVelocity_DSTATE[1];
  }

  /* End of Switch: '<S69>/UseIPPosSwitch' */

  /* Outputs for Atomic SubSystem: '<S1>/ControllerPolePlace' */
  Drone_Compe_ControllerPolePlace(rtu_pos_refin, localB->takeoff_flag,
    rtu_orient_refin, localB->controlModePosVSOrient_flagin,
    localB->UseIPPosSwitch, localB->Reshapexhat_o, localB->Reshapexhat[0],
    localB->orient_estimout, localB->acc_RS[2], localB->dorient_estimout,
    &localB->ControllerPolePlace, (P_ControllerPolePlace_Drone_C_T *)
    &localP->ControllerPolePlace, Drone_Compensator_P);

  /* End of Outputs for SubSystem: '<S1>/ControllerPolePlace' */

  /* Bias: '<S10>/Bias' */
  rtb_Ckxhatkk1_e = localB->Reshapexhat[0] + localP->Bias_Bias;

  /* Bias: '<S10>/Bias1' */
  rtb_Dk1uk1_p = localB->Reshapexhat[0] + localP->Bias1_Bias;

  /* Product: '<S37>/C[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S14>/C'
   *  Delay: '<S14>/MemoryX'
   */
  rtb_Ckxhatkk1_e = localP->C_Value[0] * localDW->MemoryX_DSTATE[0] +
    localP->C_Value[1] * localDW->MemoryX_DSTATE[1];

  /* Product: '<S37>/D[k-1]*u[k-1]' incorporates:
   *  Constant: '<S14>/D'
   */
  rtb_Dk1uk1_p = localP->D_Value * rtb_Sum[2];

  /* Outputs for Enabled SubSystem: '<S37>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S62>/Enable'
   */
  if (rtb_nicemeasurementornewupdaten) {
    /* Sum: '<S62>/Sum' incorporates:
     *  Sum: '<S37>/Add1'
     */
    rtb_r = rtb_invertzaxisGain - (rtb_Ckxhatkk1_e + rtb_Dk1uk1_p);

    /* Product: '<S62>/Product3' incorporates:
     *  Constant: '<S19>/KalmanGainL'
     */
    localB->Product3[0] = localP->KalmanGainL_Value[0] * rtb_r;
    localB->Product3[1] = localP->KalmanGainL_Value[1] * rtb_r;
  }

  /* End of Outputs for SubSystem: '<S37>/MeasurementUpdate' */

  /* Sum: '<S95>/Add1' incorporates:
   *  Constant: '<S71>/C'
   *  Constant: '<S71>/D'
   *  Delay: '<S71>/MemoryX'
   *  Product: '<S95>/C[k]*xhat[k|k-1]'
   *  Product: '<S95>/D[k-1]*u[k-1]'
   */
  rtb_Add1[0] = (localP->C_Value_j[0] * localDW->MemoryX_DSTATE_g[0] +
                 localP->C_Value_j[2] * localDW->MemoryX_DSTATE_g[1]) +
    (localP->D_Value_b[0] * rtb_Dk1uk1[0] + localP->D_Value_b[2] * rtb_Dk1uk1[1]);
  rtb_Add1[1] = (localP->C_Value_j[1] * localDW->MemoryX_DSTATE_g[0] +
                 localP->C_Value_j[3] * localDW->MemoryX_DSTATE_g[1]) +
    (localP->D_Value_b[1] * rtb_Dk1uk1[0] + localP->D_Value_b[3] * rtb_Dk1uk1[1]);

  /* Outputs for Enabled SubSystem: '<S95>/MeasurementUpdate' */

  /* Constant: '<S77>/KalmanGainL' */
  Drone_Compens_MeasurementUpdate(rtb_LogicalOperator3_j,
    localP->KalmanGainL_Value_p, &rtb_opticalFlowToVelocity_gain[0], rtb_Add1,
    &localB->MeasurementUpdate_h);

  /* End of Outputs for SubSystem: '<S95>/MeasurementUpdate' */

  /* Product: '<S95>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S71>/A'
   *  Delay: '<S71>/MemoryX'
   *  Sum: '<S95>/Add'
   */
  rtb_TmpSignalConversionAtSFun_0 = localP->A_Value_m[1] *
    localDW->MemoryX_DSTATE_g[0] + localP->A_Value_m[3] *
    localDW->MemoryX_DSTATE_g[1];

  /* Update for Delay: '<S71>/MemoryX' incorporates:
   *  Constant: '<S71>/A'
   *  Constant: '<S71>/B'
   *  Delay: '<S71>/MemoryX'
   *  Product: '<S95>/A[k]*xhat[k|k-1]'
   *  Product: '<S95>/B[k]*u[k]'
   *  Sum: '<S95>/Add'
   */
  localDW->MemoryX_DSTATE_g[0] = ((localP->B_Value_b[0] * rtb_Dk1uk1[0] +
    localP->B_Value_b[2] * rtb_Dk1uk1[1]) + (localP->A_Value_m[0] *
    localDW->MemoryX_DSTATE_g[0] + localP->A_Value_m[2] *
    localDW->MemoryX_DSTATE_g[1])) + localB->MeasurementUpdate_h.Product3[0];
  localDW->MemoryX_DSTATE_g[1] = ((localP->B_Value_b[1] * rtb_Dk1uk1[0] +
    localP->B_Value_b[3] * rtb_Dk1uk1[1]) + rtb_TmpSignalConversionAtSFun_0) +
    localB->MeasurementUpdate_h.Product3[1];

  /* Sum: '<S155>/Add1' incorporates:
   *  Constant: '<S134>/C'
   *  Constant: '<S134>/D'
   *  Delay: '<S134>/MemoryX'
   *  Product: '<S155>/C[k]*xhat[k|k-1]'
   *  Product: '<S155>/D[k-1]*u[k-1]'
   */
  rtb_Add1[0] = (localP->C_Value_f[0] * localDW->MemoryX_DSTATE_m[0] +
                 localP->C_Value_f[2] * localDW->MemoryX_DSTATE_m[1]) +
    (localP->D_Value_o[0] * rtb_vel_world[0] + localP->D_Value_o[2] *
     rtb_vel_world[1]);
  rtb_Add1[1] = (localP->C_Value_f[1] * localDW->MemoryX_DSTATE_m[0] +
                 localP->C_Value_f[3] * localDW->MemoryX_DSTATE_m[1]) +
    (localP->D_Value_o[1] * rtb_vel_world[0] + localP->D_Value_o[3] *
     rtb_vel_world[1]);

  /* Outputs for Enabled SubSystem: '<S155>/MeasurementUpdate' */

  /* Constant: '<S137>/KalmanGainL' */
  Drone_Compens_MeasurementUpdate(rtb_LogicalOperator3,
    localP->KalmanGainL_Value_o, &localB->posVIS_datin[0], rtb_Add1,
    &localB->MeasurementUpdate_o);

  /* End of Outputs for SubSystem: '<S155>/MeasurementUpdate' */

  /* Inport: '<S1>/batteryStatus_datin' */
  localB->batteryStatus_datin[0] = rtu_batteryStatus_datin[0];
  localB->batteryStatus_datin[1] = rtu_batteryStatus_datin[1];

  /* Update for DiscreteFir: '<S13>/FIR_IMUaccel' */
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

  /* End of Update for DiscreteFir: '<S13>/FIR_IMUaccel' */

  /* Update for DiscreteFilter: '<S13>/IIR_IMUgyro_r' */
  localDW->IIR_IMUgyro_r_states[4] = localDW->IIR_IMUgyro_r_states[3];
  localDW->IIR_IMUgyro_r_states[3] = localDW->IIR_IMUgyro_r_states[2];
  localDW->IIR_IMUgyro_r_states[2] = localDW->IIR_IMUgyro_r_states[1];
  localDW->IIR_IMUgyro_r_states[1] = localDW->IIR_IMUgyro_r_states[0];
  localDW->IIR_IMUgyro_r_states[0] = IIR_IMUgyro_r_tmp;

  /* Update for Delay: '<S69>/Delay' */
  localDW->Delay_DSTATE[0] = localB->UseCurrentEstimator_g.Add[0];
  localDW->Delay_DSTATE[1] = localB->UseCurrentEstimator_g.Add[1];

  /* Update for DiscreteFilter: '<S72>/IIRgyroz' */
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

  /* End of Update for DiscreteFilter: '<S72>/IIRgyroz' */

  /* Update for UnitDelay: '<S122>/UD' */
  localDW->UD_DSTATE[0] = absxk;
  localDW->UD_DSTATE[1] = t;

  /* Update for Delay: '<S68>/Delay' */
  localDW->Delay_DSTATE_b[0] = localB->Reshapexhat_o[0];
  localDW->Delay_DSTATE_b[1] = localB->Reshapexhat_o[1];

  /* Update for Delay: '<S10>/Delay2' */
  localDW->Delay2_DSTATE = localB->Reshapexhat[0];

  /* Update for DiscreteFilter: '<S15>/pressureFilter_IIR' */
  localDW->pressureFilter_IIR_states[4] = localDW->pressureFilter_IIR_states[3];
  localDW->pressureFilter_IIR_states[3] = localDW->pressureFilter_IIR_states[2];
  localDW->pressureFilter_IIR_states[2] = localDW->pressureFilter_IIR_states[1];
  localDW->pressureFilter_IIR_states[1] = localDW->pressureFilter_IIR_states[0];
  localDW->pressureFilter_IIR_states[0] = pressureFilter_IIR_tmp;

  /* Update for DiscreteFilter: '<S15>/soonarFilter_IIR' */
  localDW->soonarFilter_IIR_states[4] = localDW->soonarFilter_IIR_states[3];
  localDW->soonarFilter_IIR_states[3] = localDW->soonarFilter_IIR_states[2];
  localDW->soonarFilter_IIR_states[2] = localDW->soonarFilter_IIR_states[1];
  localDW->soonarFilter_IIR_states[1] = localDW->soonarFilter_IIR_states[0];
  localDW->soonarFilter_IIR_states[0] = soonarFilter_IIR_tmp;

  /* Update for Delay: '<S14>/MemoryX' */
  localDW->icLoad = 0U;

  /* Product: '<S37>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S14>/A'
   *  Delay: '<S14>/MemoryX'
   *  Sum: '<S37>/Add'
   */
  rtb_TmpSignalConversionAtSFun_0 = localP->A_Value[1] * localDW->
    MemoryX_DSTATE[0] + localP->A_Value[3] * localDW->MemoryX_DSTATE[1];

  /* Update for Delay: '<S14>/MemoryX' incorporates:
   *  Constant: '<S14>/A'
   *  Constant: '<S14>/B'
   *  Delay: '<S14>/MemoryX'
   *  Product: '<S37>/A[k]*xhat[k|k-1]'
   *  Product: '<S37>/B[k]*u[k]'
   *  Sum: '<S37>/Add'
   */
  localDW->MemoryX_DSTATE[0] = ((localP->A_Value[0] * localDW->MemoryX_DSTATE[0]
    + localP->A_Value[2] * localDW->MemoryX_DSTATE[1]) + localP->B_Value[0] *
    rtb_Sum[2]) + localB->Product3[0];
  localDW->MemoryX_DSTATE[1] = (localP->B_Value[1] * rtb_Sum[2] +
    rtb_TmpSignalConversionAtSFun_0) + localB->Product3[1];

  /* Update for Delay: '<S71>/MemoryX' */
  localDW->icLoad_l = 0U;

  /* Update for Delay: '<S3>/Delay1' */
  localDW->Delay1_DSTATE[0] = localB->Reshapexhat_o[0];
  localDW->Delay1_DSTATE[1] = localB->Reshapexhat_o[1];

  /* Update for Delay: '<S134>/MemoryX' */
  localDW->icLoad_j = 0U;

  /* Product: '<S155>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S134>/A'
   *  Delay: '<S134>/MemoryX'
   *  Sum: '<S155>/Add'
   */
  rtb_TmpSignalConversionAtSFun_0 = localP->A_Value_g[1] *
    localDW->MemoryX_DSTATE_m[0] + localP->A_Value_g[3] *
    localDW->MemoryX_DSTATE_m[1];

  /* Update for Delay: '<S134>/MemoryX' incorporates:
   *  Constant: '<S134>/A'
   *  Constant: '<S134>/B'
   *  Delay: '<S134>/MemoryX'
   *  Product: '<S155>/A[k]*xhat[k|k-1]'
   *  Product: '<S155>/B[k]*u[k]'
   *  Sum: '<S155>/Add'
   */
  localDW->MemoryX_DSTATE_m[0] = ((localP->B_Value_a[0] * rtb_vel_world[0] +
    localP->B_Value_a[2] * rtb_vel_world[1]) + (localP->A_Value_g[0] *
    localDW->MemoryX_DSTATE_m[0] + localP->A_Value_g[2] *
    localDW->MemoryX_DSTATE_m[1])) + localB->MeasurementUpdate_o.Product3[0];
  localDW->MemoryX_DSTATE_m[1] = ((localP->B_Value_a[1] * rtb_vel_world[0] +
    localP->B_Value_a[3] * rtb_vel_world[1]) + rtb_TmpSignalConversionAtSFun_0)
    + localB->MeasurementUpdate_o.Product3[1];

  /* Update for DiscreteIntegrator: '<S69>/SimplyIntegrateVelocity' */
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

  /* End of Update for DiscreteIntegrator: '<S69>/SimplyIntegrateVelocity' */
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
    Drone_Compensator_B->Drone_Compensator_d.ControllerPolePlace.W2ToMotorsCmd_Gain
    [0];
  Drone_Compensator_Y_motors_refout[1] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPolePlace.W2ToMotorsCmd_Gain
    [1];
  Drone_Compensator_Y_motors_refout[2] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPolePlace.W2ToMotorsCmd_Gain
    [2];
  Drone_Compensator_Y_motors_refout[3] =
    Drone_Compensator_B->Drone_Compensator_d.ControllerPolePlace.W2ToMotorsCmd_Gain
    [3];

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
  for (i = 0; i < 6; i++) {
    Drone_Compensator_Y_pose_refout[i] =
      Drone_Compensator_B->Drone_Compensator_d.ControllerPolePlace.PosVSOrient_Switch
      [i];
  }

  /* End of Outport: '<Root>/pose_refout' */

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
