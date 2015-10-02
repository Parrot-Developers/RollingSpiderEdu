/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DroneRS_Compensator.h
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

#ifndef RTW_HEADER_DroneRS_Compensator_h_
#define RTW_HEADER_DroneRS_Compensator_h_
#include <stddef.h>
#include <math.h>
#include <float.h>
#include <string.h>
#ifndef DroneRS_Compensator_COMMON_INCLUDES_
# define DroneRS_Compensator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_logging.h"
#endif                                 /* DroneRS_Compensator_COMMON_INCLUDES_ */

#include "DroneRS_Compensator_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals for system '<S1>/ControllerPID2W' */
typedef struct {
  real_T pos_ref[3];                   /* '<S2>/pos_ref' */
  real_T att_ref[3];                   /* '<S2>/att_ref' */
  real_T Switch_refAtt[2];             /* '<S2>/Switch_refAtt' */
  real_T Product1[4];                  /* '<S6>/Product1' */
} B_ControllerPID2W_DroneRS_Com_T;

/* Block states (auto storage) for system '<S1>/ControllerPID2W' */
typedef struct {
  real_T Delay_DSTATE[2];              /* '<S2>/Delay' */
  real_T DiscreteTimeIntegrator_DSTATE[2];/* '<S2>/Discrete-Time Integrator' */
  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<S2>/To Workspace1' */

  struct {
    void *LoggedData;
  } thrustw_PWORK;                     /* '<S2>/thrustw' */
} DW_ControllerPID2W_DroneRS_Co_T;

/* Block signals for system '<S89>/MeasurementUpdate' */
typedef struct {
  real_T Product3[2];                  /* '<S114>/Product3' */
} B_MeasurementUpdate_DroneRS_C_T;

/* Block signals for system '<S68>/UseCurrentEstimator' */
typedef struct {
  real_T Add[2];                       /* '<S94>/Add' */
  real_T Product2[2];                  /* '<S115>/Product2' */
} B_UseCurrentEstimator_DroneRS_T;

/* Block signals for system '<Root>/DroneRS_Compensator' */
typedef struct {
  real_T posVIS_datin[4];              /* '<S1>/posVIS_datin' */
  real_T sensordatabiasRS_datin[7];    /* '<S1>/sensordatabiasRS_datin' */
  real_T sensordataRS_datin[8];        /* '<S1>/sensordataRS_datin' */
  real_T usePosVIS_flagin;             /* '<S1>/usePosVIS_flagin' */
  real_T opticalFlowRS_datin[3];       /* '<S1>/opticalFlowRS_datin' */
  real_T FIRaccelero[3];               /* '<S10>/FIRaccelero' */
  real_T Reshapexhat[2];               /* '<S11>/Reshapexhat' */
  real_T Reshapexhat_o[2];             /* '<S68>/Reshapexhat' */
  real_T UseIPPosSwitch[2];            /* '<S66>/UseIPPosSwitch' */
  real_T batteryStatus_datin[2];       /* '<S1>/batteryStatus_datin' */
  real_T att_estimout[3];              /* '<S3>/EstimatorAttitude' */
  real_T datt_estimout[3];             /* '<S3>/EstimatorAttitude' */
  real_T acc_RS[3];                    /* '<S7>/WorldToRSinacc' */
  real_T Product2[2];                  /* '<S60>/Product2' */
  real_T Product3[2];                  /* '<S59>/Product3' */
  boolean_T controlModePosVSAtt_flagin;/* '<S1>/controlModePosVSAtt_flagin' */
  B_UseCurrentEstimator_DroneRS_T UseCurrentEstimator_f;/* '<S128>/UseCurrentEstimator' */
  B_MeasurementUpdate_DroneRS_C_T MeasurementUpdate_f;/* '<S149>/MeasurementUpdate' */
  B_UseCurrentEstimator_DroneRS_T UseCurrentEstimator_l;/* '<S68>/UseCurrentEstimator' */
  B_MeasurementUpdate_DroneRS_C_T MeasurementUpdate_p;/* '<S89>/MeasurementUpdate' */
  B_ControllerPID2W_DroneRS_Com_T ControllerPID2W;/* '<S1>/ControllerPID2W' */
} B_DroneRS_Compensator_DroneRS_T;

/* Block states (auto storage) for system '<Root>/DroneRS_Compensator' */
typedef struct {
  real_T FIRaccelero_states[15];       /* '<S10>/FIRaccelero' */
  real_T IIRgyroz_states[5];           /* '<S10>/IIRgyroz' */
  real_T Delay_DSTATE[2];              /* '<S66>/Delay' */
  real_T IIRgyroz_states_n[10];        /* '<S69>/IIRgyroz' */
  real_T UD_DSTATE[2];                 /* '<S116>/UD' */
  real_T Delay2_DSTATE;                /* '<S7>/Delay2' */
  real_T IIRprs_states[5];             /* '<S12>/IIRprs' */
  real_T IIRsonar_states[5];           /* '<S12>/IIRsonar' */
  real_T MemoryX_DSTATE[2];            /* '<S11>/MemoryX' */
  real_T Delay_DSTATE_l[2];            /* '<S65>/Delay' */
  real_T MemoryX_DSTATE_f[2];          /* '<S68>/MemoryX' */
  real_T Delay1_DSTATE[2];             /* '<S3>/Delay1' */
  real_T MemoryX_DSTATE_a[2];          /* '<S128>/MemoryX' */
  real_T SimplyIntegrateVelocity_DSTATE[2];/* '<S66>/SimplyIntegrateVelocity' */
  real_T IIRgyroz_tmp_f[2];            /* '<S69>/IIRgyroz' */
  real_T yaw_cur;                      /* '<S3>/EstimatorAttitude' */
  real_T pitch_cur;                    /* '<S3>/EstimatorAttitude' */
  real_T roll_cur;                     /* '<S3>/EstimatorAttitude' */
  struct {
    void *LoggedData;
  } SCOPE_anglesRSestim_PWORK;         /* '<S3>/SCOPE_anglesRSestim' */

  struct {
    void *LoggedData;
  } eulerrates_PWORK;                  /* '<S3>/eulerrates' */

  struct {
    void *LoggedData;
  } SCOPE_altPrs_PWORK;                /* '<S7>/SCOPE_altPrs' */

  struct {
    void *LoggedData;
  } SCOPE_kalmanaltestim_PWORK;        /* '<S7>/SCOPE_kalmanaltestim' */

  struct {
    void *LoggedData;
  } SCOPEdxy_PWORK;                    /* '<S65>/SCOPEdxy' */

  struct {
    void *LoggedData;
  } SCOPEenableKFdxupdate_PWORK;       /* '<S65>/SCOPEenableKFdxupdate' */

  int32_T FIRaccelero_circBuf;         /* '<S10>/FIRaccelero' */
  int8_T SimplyIntegrateVelocity_PrevRes;/* '<S66>/SimplyIntegrateVelocity' */
  uint8_T icLoad;                      /* '<S11>/MemoryX' */
  uint8_T icLoad_c;                    /* '<S68>/MemoryX' */
  uint8_T icLoad_e;                    /* '<S128>/MemoryX' */
  DW_ControllerPID2W_DroneRS_Co_T ControllerPID2W;/* '<S1>/ControllerPID2W' */
} DW_DroneRS_Compensator_DroneR_T;

/* Block signals (auto storage) */
typedef struct {
  B_DroneRS_Compensator_DroneRS_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
} B_DroneRS_Compensator_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  DW_DroneRS_Compensator_DroneR_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
} DW_DroneRS_Compensator_T;

/* Parameters for system: '<S1>/ControllerPID2W' */
struct P_ControllerPID2W_DroneRS_Com_T_ {
  real_T D_z_Gain;                     /* Expression: 350
                                        * Referenced by: '<S2>/D_z'
                                        */
  real_T P_z_Gain;                     /* Expression: 600
                                        * Referenced by: '<S2>/P_z'
                                        */
  real_T takeoff_Gain_Gain;            /* Expression: 0.05
                                        * Referenced by: '<S2>/takeoff_Gain'
                                        */
  real_T D_xy_Gain[2];                 /* Expression: [0.08, -0.09]
                                        * Referenced by: '<S2>/D_xy'
                                        */
  real_T P_xy_Gain[2];                 /* Expression: [-0.32,0.29]
                                        * Referenced by: '<S2>/P_xy'
                                        */
  real_T TakeoffOrControl_Switch_Thresho;/* Expression: 0
                                          * Referenced by: '<S2>/TakeoffOrControl_Switch'
                                          */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S2>/Delay'
                                        */
  real_T antiWU_Gain_Gain;             /* Expression: 0.001
                                        * Referenced by: '<S2>/antiWU_Gain'
                                        */
  real_T Action2omega_Value[16];       /* Expression: [0.7071, 0.7071, -1.0000, -1.00;   -0.7071,    0.7071,   -1.0000,    1.00;   -0.7071,   -0.7071,   -1.0000,   -1.00;    0.7071,   -0.7071,   -1.0000,    1.00]
                                        * Referenced by: '<S4>/Action2omega'
                                        */
  real_T P_pr_Gain[2];                 /* Expression: [600;1300]
                                        * Referenced by: '<S2>/P_pr'
                                        */
  real_T DiscreteTimeIntegrator_gainval;/* Computed Parameter: DiscreteTimeIntegrator_gainval
                                         * Referenced by: '<S2>/Discrete-Time Integrator'
                                         */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S2>/Discrete-Time Integrator'
                                        */
  real_T DiscreteTimeIntegrator_UpperSat;/* Expression: 2
                                          * Referenced by: '<S2>/Discrete-Time Integrator'
                                          */
  real_T DiscreteTimeIntegrator_LowerSat;/* Expression: -2
                                          * Referenced by: '<S2>/Discrete-Time Integrator'
                                          */
  real_T I_pr_Gain;                    /* Expression: 200
                                        * Referenced by: '<S2>/I_pr'
                                        */
  real_T D_pr_Gain[2];                 /* Expression: [80;300]
                                        * Referenced by: '<S2>/D_pr'
                                        */
  real_T P_yaw_Gain;                   /* Expression: 6000
                                        * Referenced by: '<S2>/P_yaw'
                                        */
  real_T D_yaw_Gain;                   /* Expression: 0.3*6000
                                        * Referenced by: '<S2>/D_yaw'
                                        */
  real_T SaturationThrust_UpperSat;    /* Expression: 0.92*2618
                                        * Referenced by: '<S2>/SaturationThrust'
                                        */
  real_T SaturationThrust_LowerSat;    /* Expression: -0.92*2618
                                        * Referenced by: '<S2>/SaturationThrust'
                                        */
  real_T Gain3_Gain;                   /* Expression: -1
                                        * Referenced by: '<S4>/Gain3'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 2618
                                        * Referenced by: '<S4>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 500
                                        * Referenced by: '<S4>/Saturation1'
                                        */
  real_T Gain4_Gain;                   /* Expression: -1
                                        * Referenced by: '<S4>/Gain4'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<S4>/Gain5'
                                        */
  real_T Saturation4_UpperSat;         /* Expression: 2618
                                        * Referenced by: '<S4>/Saturation4'
                                        */
  real_T Saturation4_LowerSat;         /* Expression: 500
                                        * Referenced by: '<S4>/Saturation4'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<S4>/Gain6'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 2618
                                        * Referenced by: '<S4>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: 500
                                        * Referenced by: '<S4>/Saturation2'
                                        */
  real_T Saturation3_UpperSat;         /* Expression: 2618
                                        * Referenced by: '<S4>/Saturation3'
                                        */
  real_T Saturation3_LowerSat;         /* Expression: 500
                                        * Referenced by: '<S4>/Saturation3'
                                        */
  real_T w2MotorsQF_Gain_Gain;         /* Expression: 0.0085
                                        * Referenced by: '<S6>/w2MotorsQF_Gain'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S2>/Delay'
                                        */
};

/* Parameters for system: '<Root>/DroneRS_Compensator' */
struct P_DroneRS_Compensator_DroneRS_T_ {
  real_T DiscreteDerivative_ICPrevScaled;/* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                          * Referenced by: '<S116>/UD'
                                          */
  real_T checkPosavailable_const;      /* Mask Parameter: checkPosavailable_const
                                        * Referenced by: '<S177>/Constant'
                                        */
  real_T CompareToConstant_const;      /* Mask Parameter: CompareToConstant_const
                                        * Referenced by: '<S181>/Constant'
                                        */
  real_T outlierBelowFloor_const;      /* Mask Parameter: outlierBelowFloor_const
                                        * Referenced by: '<S15>/Constant'
                                        */
  real_T FIRaccelero_InitialStates;    /* Expression: 0
                                        * Referenced by: '<S10>/FIRaccelero'
                                        */
  real_T FIRaccelero_Coefficients[6];  /* Expression: controlParams.filter_accelero.Coefficients
                                        * Referenced by: '<S10>/FIRaccelero'
                                        */
  real_T IIRgyroz_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S10>/IIRgyroz'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S66>/Delay'
                                        */
  real_T KalmanGainM_Value[4];         /* Expression: pInitialization.M
                                        * Referenced by: '<S131>/KalmanGainM'
                                        */
  real_T IIRgyroz_InitialStates_c;     /* Expression: 0
                                        * Referenced by: '<S69>/IIRgyroz'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S116>/TSamp'
                                        */
  real_T invertzaxisGain_Gain;         /* Expression: -1
                                        * Referenced by: '<S7>/invertzaxisGain'
                                        */
  real_T SaturationSonar_LowerSat;     /* Expression: -inf
                                        * Referenced by: '<S12>/SaturationSonar'
                                        */
  real_T Delay2_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S7>/Delay2'
                                        */
  real_T IIRprs_InitialStates;         /* Expression: 0
                                        * Referenced by: '<S12>/IIRprs'
                                        */
  real_T IIRsonar_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S12>/IIRsonar'
                                        */
  real_T KalmanGainM_Value_h[2];       /* Expression: pInitialization.M
                                        * Referenced by: '<S16>/KalmanGainM'
                                        */
  real_T gravity_Value[3];             /* Expression: [0 0 quad.g]
                                        * Referenced by: '<S7>/gravity'
                                        */
  real_T C_Value[2];                   /* Expression: pInitialization.C
                                        * Referenced by: '<S11>/C'
                                        */
  real_T D_Value;                      /* Expression: pInitialization.D
                                        * Referenced by: '<S11>/D'
                                        */
  real_T X0_Value[2];                  /* Expression: pInitialization.X0
                                        * Referenced by: '<S11>/X0'
                                        */
  real_T Delay_InitialCondition_n;     /* Expression: 0
                                        * Referenced by: '<S65>/Delay'
                                        */
  real_T KalmanGainM_Value_f[4];       /* Expression: pInitialization.M
                                        * Referenced by: '<S71>/KalmanGainM'
                                        */
  real_T gravity_Value_g[3];           /* Expression: [0 0 -quad.g]
                                        * Referenced by: '<S67>/gravity'
                                        */
  real_T gainaccinput_Gain;            /* Expression: 0.2
                                        * Referenced by: '<S67>/gainaccinput'
                                        */
  real_T C_Value_d[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S68>/C'
                                        */
  real_T D_Value_f[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S68>/D'
                                        */
  real_T X0_Value_k[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S68>/X0'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S3>/Delay1'
                                        */
  real_T C_Value_f[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S128>/C'
                                        */
  real_T D_Value_f0[4];                /* Expression: pInitialization.D
                                        * Referenced by: '<S128>/D'
                                        */
  real_T X0_Value_d[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S128>/X0'
                                        */
  real_T SimplyIntegrateVelocity_gainval;/* Computed Parameter: SimplyIntegrateVelocity_gainval
                                          * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_IC;   /* Expression: 0
                                        * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                        */
  real_T SimplyIntegrateVelocity_UpperSa;/* Expression: 2
                                          * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_LowerSa;/* Expression: -2
                                          * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                          */
  real_T UseIPPosSwitch_Threshold;     /* Expression: 0
                                        * Referenced by: '<S66>/UseIPPosSwitch'
                                        */
  real_T A_Value[4];                   /* Expression: pInitialization.A
                                        * Referenced by: '<S11>/A'
                                        */
  real_T B_Value[2];                   /* Expression: pInitialization.B
                                        * Referenced by: '<S11>/B'
                                        */
  real_T KalmanGainL_Value[2];         /* Expression: pInitialization.L
                                        * Referenced by: '<S16>/KalmanGainL'
                                        */
  real_T A_Value_d[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S68>/A'
                                        */
  real_T B_Value_c[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S68>/B'
                                        */
  real_T KalmanGainL_Value_f[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S71>/KalmanGainL'
                                        */
  real_T A_Value_dj[4];                /* Expression: pInitialization.A
                                        * Referenced by: '<S128>/A'
                                        */
  real_T B_Value_k[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S128>/B'
                                        */
  real_T KalmanGainL_Value_j[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S131>/KalmanGainL'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S66>/Delay'
                                        */
  uint32_T Delay2_DelayLength;         /* Computed Parameter: Delay2_DelayLength
                                        * Referenced by: '<S7>/Delay2'
                                        */
  uint32_T MemoryX_DelayLength;        /* Computed Parameter: MemoryX_DelayLength
                                        * Referenced by: '<S11>/MemoryX'
                                        */
  uint32_T Delay_DelayLength_g;        /* Computed Parameter: Delay_DelayLength_g
                                        * Referenced by: '<S65>/Delay'
                                        */
  uint32_T MemoryX_DelayLength_g;      /* Computed Parameter: MemoryX_DelayLength_g
                                        * Referenced by: '<S68>/MemoryX'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S3>/Delay1'
                                        */
  uint32_T MemoryX_DelayLength_m;      /* Computed Parameter: MemoryX_DelayLength_m
                                        * Referenced by: '<S128>/MemoryX'
                                        */
  P_ControllerPID2W_DroneRS_Com_T ControllerPID2W;/* '<S1>/ControllerPID2W' */
};

/* Parameters (auto storage) */
struct P_DroneRS_Compensator_T_ {
  struct_nVjCgugzLFJzCZr6yyeDeH quadEDT;/* Variable: quadEDT
                                         * Referenced by:
                                         *   '<S7>/prsToAlt_Gain'
                                         *   '<S10>/inversesIMU_Gain'
                                         *   '<S12>/SaturationSonar'
                                         *   '<S65>/opticalFlowToVelocity_Gain'
                                         *   '<S63>/Constant'
                                         */
  struct_pP0yJPvqYhejK9gHgcbWI quad;   /* Variable: quad
                                        * Referenced by: '<S2>/w0'
                                        */
  struct_eTOByJ6BrrCe8gZfBpKFUD altEstim;/* Variable: altEstim
                                          * Referenced by:
                                          *   '<S7>/Bias'
                                          *   '<S7>/Bias1'
                                          *   '<S10>/IIRgyroz'
                                          *   '<S12>/IIRprs'
                                          *   '<S12>/IIRsonar'
                                          *   '<S61>/Constant'
                                          *   '<S62>/Constant'
                                          *   '<S64>/Constant'
                                          *   '<S69>/IIRgyroz'
                                          */
  struct_rM3FFntOU5Aaym8djgtmlC ofhandle;/* Variable: ofhandle
                                          * Referenced by:
                                          *   '<S117>/Constant'
                                          *   '<S118>/Constant'
                                          *   '<S119>/Constant'
                                          *   '<S120>/Constant'
                                          *   '<S121>/Constant'
                                          *   '<S122>/Constant'
                                          *   '<S123>/Constant'
                                          *   '<S124>/Constant'
                                          *   '<S125>/Constant'
                                          *   '<S126>/Constant'
                                          *   '<S127>/Constant'
                                          */
  struct_YkbJnRR8M5ye4XtO88GdQC vishandle;/* Variable: vishandle
                                           * Referenced by:
                                           *   '<S178>/Constant'
                                           *   '<S179>/Constant'
                                           *   '<S180>/Constant'
                                           */
  real_T sampleTime_qcsim;             /* Variable: sampleTime_qcsim
                                        * Referenced by: '<S3>/sampleTime'
                                        */
  P_DroneRS_Compensator_DroneRS_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
};

/* Real-time Model Data Structure */
struct tag_RTM_DroneRS_Compensator_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * ModelData:
   * The following substructure contains information regarding
   * the data used in the model.
   */
  struct {
    B_DroneRS_Compensator_T *blockIO;
    P_DroneRS_Compensator_T *defaultParam;
    DW_DroneRS_Compensator_T *dwork;
  } ModelData;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Model entry point functions */
extern void DroneRS_Compensator_initialize(RT_MODEL_DroneRS_Compensator_T *const
  DroneRS_Compensator_M, boolean_T
  *DroneRS_Compensator_U_controlModePosVSAtt_flagin, real_T
  DroneRS_Compensator_U_pos_refin[3], real_T DroneRS_Compensator_U_attRS_refin[3],
  real_T *DroneRS_Compensator_U_ddx, real_T *DroneRS_Compensator_U_ddy, real_T
  *DroneRS_Compensator_U_ddz, real_T *DroneRS_Compensator_U_p, real_T
  *DroneRS_Compensator_U_q, real_T *DroneRS_Compensator_U_r, real_T
  *DroneRS_Compensator_U_altitude_sonar, real_T *DroneRS_Compensator_U_prs,
  real_T DroneRS_Compensator_U_opticalFlowRS_datin[3], real_T
  DroneRS_Compensator_U_sensordatabiasRS_datin[7], real_T
  DroneRS_Compensator_U_posVIS_datin[4], real_T
  *DroneRS_Compensator_U_usePosVIS_flagin, real_T
  DroneRS_Compensator_U_batteryStatus_datin[2], real_T
  DroneRS_Compensator_Y_motorsRS_cmdout[4], real_T *DroneRS_Compensator_Y_X,
  real_T *DroneRS_Compensator_Y_Y, real_T *DroneRS_Compensator_Y_Z, real_T
  *DroneRS_Compensator_Y_yaw, real_T *DroneRS_Compensator_Y_pitch, real_T
  *DroneRS_Compensator_Y_roll, real_T *DroneRS_Compensator_Y_dx, real_T
  *DroneRS_Compensator_Y_dy, real_T *DroneRS_Compensator_Y_dz, real_T
  *DroneRS_Compensator_Y_pb, real_T *DroneRS_Compensator_Y_qb, real_T
  *DroneRS_Compensator_Y_rb, boolean_T
  *DroneRS_Compensator_Y_controlModePosVSAtt_flagout, real_T
  DroneRS_Compensator_Y_poseRS_refout[6], real_T *DroneRS_Compensator_Y_ddxb,
  real_T *DroneRS_Compensator_Y_ddyb, real_T *DroneRS_Compensator_Y_ddzb, real_T
  *DroneRS_Compensator_Y_pa, real_T *DroneRS_Compensator_Y_qa, real_T
  *DroneRS_Compensator_Y_ra, real_T *DroneRS_Compensator_Y_altitude_sonarb,
  real_T *DroneRS_Compensator_Y_prsb, real_T
  DroneRS_Compensator_Y_opticalFlowRS_datout[3], real_T
  DroneRS_Compensator_Y_sensordatabiasRS_datout[7], real_T
  DroneRS_Compensator_Y_posVIS_datout[4], real_T
  *DroneRS_Compensator_Y_usePosVIS_flagout, real_T
  DroneRS_Compensator_Y_batteryStatus_datout[2]);
extern void DroneRS_Compensator_step(RT_MODEL_DroneRS_Compensator_T *const
  DroneRS_Compensator_M, boolean_T
  DroneRS_Compensator_U_controlModePosVSAtt_flagin, real_T
  DroneRS_Compensator_U_pos_refin[3], real_T DroneRS_Compensator_U_attRS_refin[3],
  real_T DroneRS_Compensator_U_ddx, real_T DroneRS_Compensator_U_ddy, real_T
  DroneRS_Compensator_U_ddz, real_T DroneRS_Compensator_U_p, real_T
  DroneRS_Compensator_U_q, real_T DroneRS_Compensator_U_r, real_T
  DroneRS_Compensator_U_altitude_sonar, real_T DroneRS_Compensator_U_prs, real_T
  DroneRS_Compensator_U_opticalFlowRS_datin[3], real_T
  DroneRS_Compensator_U_sensordatabiasRS_datin[7], real_T
  DroneRS_Compensator_U_posVIS_datin[4], real_T
  DroneRS_Compensator_U_usePosVIS_flagin, real_T
  DroneRS_Compensator_U_batteryStatus_datin[2], real_T
  DroneRS_Compensator_Y_motorsRS_cmdout[4], real_T *DroneRS_Compensator_Y_X,
  real_T *DroneRS_Compensator_Y_Y, real_T *DroneRS_Compensator_Y_Z, real_T
  *DroneRS_Compensator_Y_yaw, real_T *DroneRS_Compensator_Y_pitch, real_T
  *DroneRS_Compensator_Y_roll, real_T *DroneRS_Compensator_Y_dx, real_T
  *DroneRS_Compensator_Y_dy, real_T *DroneRS_Compensator_Y_dz, real_T
  *DroneRS_Compensator_Y_pb, real_T *DroneRS_Compensator_Y_qb, real_T
  *DroneRS_Compensator_Y_rb, boolean_T
  *DroneRS_Compensator_Y_controlModePosVSAtt_flagout, real_T
  DroneRS_Compensator_Y_poseRS_refout[6], real_T *DroneRS_Compensator_Y_ddxb,
  real_T *DroneRS_Compensator_Y_ddyb, real_T *DroneRS_Compensator_Y_ddzb, real_T
  *DroneRS_Compensator_Y_pa, real_T *DroneRS_Compensator_Y_qa, real_T
  *DroneRS_Compensator_Y_ra, real_T *DroneRS_Compensator_Y_altitude_sonarb,
  real_T *DroneRS_Compensator_Y_prsb, real_T
  DroneRS_Compensator_Y_opticalFlowRS_datout[3], real_T
  DroneRS_Compensator_Y_sensordatabiasRS_datout[7], real_T
  DroneRS_Compensator_Y_posVIS_datout[4], real_T
  *DroneRS_Compensator_Y_usePosVIS_flagout, real_T
  DroneRS_Compensator_Y_batteryStatus_datout[2]);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('sim_quadrotor/DroneRS_Compensator')    - opens subsystem sim_quadrotor/DroneRS_Compensator
 * hilite_system('sim_quadrotor/DroneRS_Compensator/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'sim_quadrotor'
 * '<S1>'   : 'sim_quadrotor/DroneRS_Compensator'
 * '<S2>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerPID2W'
 * '<S3>'   : 'sim_quadrotor/DroneRS_Compensator/Estimator'
 * '<S4>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerPID2W/Control ControlMixerRS'
 * '<S5>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerPID2W/inverse rotation Function'
 * '<S6>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerPID2W/wToMotors'
 * '<S7>'   : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude'
 * '<S8>'   : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAttitude'
 * '<S9>'   : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition'
 * '<S10>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/SensorPreprocessing'
 * '<S11>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude'
 * '<S12>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling'
 * '<S13>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/RStoWorldinacc'
 * '<S14>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/WorldToRSinacc'
 * '<S15>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/outlierBelowFloor'
 * '<S16>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL'
 * '<S17>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculateYhat'
 * '<S18>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionA'
 * '<S19>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionB'
 * '<S20>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionC'
 * '<S21>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionD'
 * '<S22>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionG'
 * '<S23>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionH'
 * '<S24>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionN'
 * '<S25>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP'
 * '<S26>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP0'
 * '<S27>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionQ'
 * '<S28>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionR'
 * '<S29>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionReset'
 * '<S30>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX'
 * '<S31>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX0'
 * '<S32>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionu'
 * '<S33>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/MemoryP'
 * '<S34>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer'
 * '<S35>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ReducedQRN'
 * '<S36>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionP0'
 * '<S37>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionQ'
 * '<S38>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionR'
 * '<S39>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator'
 * '<S40>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkA'
 * '<S41>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkB'
 * '<S42>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkC'
 * '<S43>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkD'
 * '<S44>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkEnable'
 * '<S45>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkG'
 * '<S46>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkH'
 * '<S47>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkN'
 * '<S48>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkP0'
 * '<S49>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkQ'
 * '<S50>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkR'
 * '<S51>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkReset'
 * '<S52>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkX0'
 * '<S53>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checku'
 * '<S54>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checky'
 * '<S55>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionL'
 * '<S56>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionM'
 * '<S57>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionP'
 * '<S58>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionZ'
 * '<S59>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer/MeasurementUpdate'
 * '<S60>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator/Enabled Subsystem'
 * '<S61>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffprs'
 * '<S62>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffsonarflt'
 * '<S63>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierDist_min'
 * '<S64>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierJump'
 * '<S65>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity'
 * '<S66>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition'
 * '<S67>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld'
 * '<S68>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy'
 * '<S69>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling'
 * '<S70>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld/World2Body'
 * '<S71>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL'
 * '<S72>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculateYhat'
 * '<S73>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionA'
 * '<S74>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionB'
 * '<S75>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionC'
 * '<S76>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionD'
 * '<S77>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionG'
 * '<S78>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionH'
 * '<S79>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionN'
 * '<S80>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP'
 * '<S81>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP0'
 * '<S82>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionQ'
 * '<S83>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionR'
 * '<S84>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionReset'
 * '<S85>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX'
 * '<S86>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX0'
 * '<S87>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionu'
 * '<S88>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/MemoryP'
 * '<S89>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer'
 * '<S90>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ReducedQRN'
 * '<S91>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionP0'
 * '<S92>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionQ'
 * '<S93>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionR'
 * '<S94>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator'
 * '<S95>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkA'
 * '<S96>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkB'
 * '<S97>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkC'
 * '<S98>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkD'
 * '<S99>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkEnable'
 * '<S100>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkG'
 * '<S101>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkH'
 * '<S102>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkN'
 * '<S103>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkP0'
 * '<S104>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkQ'
 * '<S105>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkR'
 * '<S106>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkReset'
 * '<S107>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkX0'
 * '<S108>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checku'
 * '<S109>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checky'
 * '<S110>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionL'
 * '<S111>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionM'
 * '<S112>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionP'
 * '<S113>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionZ'
 * '<S114>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer/MeasurementUpdate'
 * '<S115>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator/Enabled Subsystem'
 * '<S116>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/DiscreteDerivative'
 * '<S117>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxdw1'
 * '<S118>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxdw2'
 * '<S119>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxp'
 * '<S120>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxp2'
 * '<S121>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxq'
 * '<S122>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxq2'
 * '<S123>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw1'
 * '<S124>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw2'
 * '<S125>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw3'
 * '<S126>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw4'
 * '<S127>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/minHeightforOF'
 * '<S128>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy'
 * '<S129>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling'
 * '<S130>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/RStoWorld'
 * '<S131>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL'
 * '<S132>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculateYhat'
 * '<S133>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionA'
 * '<S134>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionB'
 * '<S135>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionC'
 * '<S136>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionD'
 * '<S137>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionG'
 * '<S138>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionH'
 * '<S139>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionN'
 * '<S140>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP'
 * '<S141>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP0'
 * '<S142>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionQ'
 * '<S143>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionR'
 * '<S144>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionReset'
 * '<S145>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX'
 * '<S146>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX0'
 * '<S147>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionu'
 * '<S148>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/MemoryP'
 * '<S149>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer'
 * '<S150>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ReducedQRN'
 * '<S151>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionP0'
 * '<S152>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionQ'
 * '<S153>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionR'
 * '<S154>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator'
 * '<S155>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkA'
 * '<S156>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkB'
 * '<S157>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkC'
 * '<S158>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkD'
 * '<S159>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkEnable'
 * '<S160>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkG'
 * '<S161>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkH'
 * '<S162>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkN'
 * '<S163>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkP0'
 * '<S164>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkQ'
 * '<S165>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkR'
 * '<S166>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkReset'
 * '<S167>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkX0'
 * '<S168>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checku'
 * '<S169>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checky'
 * '<S170>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionL'
 * '<S171>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionM'
 * '<S172>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionP'
 * '<S173>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionZ'
 * '<S174>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer/MeasurementUpdate'
 * '<S175>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator/Enabled Subsystem'
 * '<S176>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs'
 * '<S177>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/checkPosavailable'
 * '<S178>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxp3'
 * '<S179>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxq3'
 * '<S180>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/planarjumpsVISPOS'
 * '<S181>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/SensorPreprocessing/Compare To Constant'
 */
#endif                                 /* RTW_HEADER_DroneRS_Compensator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
