/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DroneRS_Compensator.h
 *
 * Code generated for Simulink model 'DroneRS_Compensator'.
 *
 * Model version                  : 1.2611
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Oct 14 16:32:57 2015
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
#include <math.h>
#include <float.h>
#include <stddef.h>
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

/* Block signals for system '<S1>/ControllerFSFB' */
typedef struct {
  real_T PosVSAtt_Switch[12];          /* '<S6>/PosVSAtt_Switch' */
  real_T W2ToMotorsCmd_Gain[4];        /* '<S9>/W2ToMotorsCmd_Gain' */
} B_ControllerFSFB_DroneRS_Comp_T;

/* Block states (auto storage) for system '<S1>/ControllerFSFB' */
typedef struct {
  struct {
    void *LoggedData;
  } SCOPE_totalthrust_PWORK;           /* '<S8>/SCOPE_totalthrust ' */
} DW_ControllerFSFB_DroneRS_Com_T;

/* Block signals for system '<S93>/MeasurementUpdate' */
typedef struct {
  real_T Product3[2];                  /* '<S118>/Product3' */
} B_MeasurementUpdate_DroneRS_C_T;

/* Block signals for system '<S72>/UseCurrentEstimator' */
typedef struct {
  real_T Add[2];                       /* '<S98>/Add' */
  real_T Product2[2];                  /* '<S119>/Product2' */
} B_UseCurrentEstimator_DroneRS_T;

/* Block signals for system '<Root>/DroneRS_Compensator' */
typedef struct {
  real_T posVIS_datin[4];              /* '<S1>/posVIS_datin' */
  real_T sensordatabiasRS_datin[7];    /* '<S1>/sensordatabiasRS_datin' */
  real_T sensordataRS_datin[8];        /* '<S1>/sensordataRS_datin' */
  real_T usePosVIS_flagin;             /* '<S1>/usePosVIS_flagin' */
  real_T opticalFlowRS_datin[3];       /* '<S1>/opticalFlowRS_datin' */
  real_T FIRaccelero[3];               /* '<S14>/FIRaccelero' */
  real_T Reshapexhat[2];               /* '<S15>/Reshapexhat' */
  real_T Reshapexhat_o[2];             /* '<S72>/Reshapexhat' */
  real_T UseIPPosSwitch[2];            /* '<S70>/UseIPPosSwitch' */
  real_T batteryStatus_datin[2];       /* '<S1>/batteryStatus_datin' */
  real_T att_estimout[3];              /* '<S3>/EstimatorAttitude' */
  real_T datt_estimout[3];             /* '<S3>/EstimatorAttitude' */
  real_T acc_RS[3];                    /* '<S11>/WorldToRSinacc' */
  real_T Product2[2];                  /* '<S64>/Product2' */
  real_T Product3[2];                  /* '<S63>/Product3' */
  boolean_T controlModePosVSAtt_flagin;/* '<S1>/controlModePosVSAtt_flagin' */
  B_UseCurrentEstimator_DroneRS_T UseCurrentEstimator_f;/* '<S132>/UseCurrentEstimator' */
  B_MeasurementUpdate_DroneRS_C_T MeasurementUpdate_f;/* '<S153>/MeasurementUpdate' */
  B_UseCurrentEstimator_DroneRS_T UseCurrentEstimator_l;/* '<S72>/UseCurrentEstimator' */
  B_MeasurementUpdate_DroneRS_C_T MeasurementUpdate_p;/* '<S93>/MeasurementUpdate' */
  B_ControllerFSFB_DroneRS_Comp_T ControllerFSFB;/* '<S1>/ControllerFSFB' */
} B_DroneRS_Compensator_DroneRS_T;

/* Block states (auto storage) for system '<Root>/DroneRS_Compensator' */
typedef struct {
  real_T FIRaccelero_states[15];       /* '<S14>/FIRaccelero' */
  real_T IIRgyroz_states[5];           /* '<S14>/IIRgyroz' */
  real_T Delay_DSTATE[2];              /* '<S70>/Delay' */
  real_T IIRgyroz_states_n[10];        /* '<S73>/IIRgyroz' */
  real_T UD_DSTATE[2];                 /* '<S120>/UD' */
  real_T Delay2_DSTATE;                /* '<S11>/Delay2' */
  real_T IIRprs_states[5];             /* '<S16>/IIRprs' */
  real_T IIRsonar_states[5];           /* '<S16>/IIRsonar' */
  real_T MemoryX_DSTATE[2];            /* '<S15>/MemoryX' */
  real_T Delay_DSTATE_l[2];            /* '<S69>/Delay' */
  real_T MemoryX_DSTATE_f[2];          /* '<S72>/MemoryX' */
  real_T Delay1_DSTATE[2];             /* '<S3>/Delay1' */
  real_T MemoryX_DSTATE_a[2];          /* '<S132>/MemoryX' */
  real_T SimplyIntegrateVelocity_DSTATE[2];/* '<S70>/SimplyIntegrateVelocity' */
  real_T IIRgyroz_tmp_f[2];            /* '<S73>/IIRgyroz' */
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
  } SCOPE_altPrs_PWORK;                /* '<S11>/SCOPE_altPrs' */

  struct {
    void *LoggedData;
  } SCOPE_kalmanaltestim_PWORK;        /* '<S11>/SCOPE_kalmanaltestim' */

  struct {
    void *LoggedData;
  } SCOPEdxy_PWORK;                    /* '<S69>/SCOPEdxy' */

  struct {
    void *LoggedData;
  } SCOPEenableKFdxupdate_PWORK;       /* '<S69>/SCOPEenableKFdxupdate' */

  int32_T FIRaccelero_circBuf;         /* '<S14>/FIRaccelero' */
  int8_T SimplyIntegrateVelocity_PrevRes;/* '<S70>/SimplyIntegrateVelocity' */
  uint8_T icLoad;                      /* '<S15>/MemoryX' */
  uint8_T icLoad_c;                    /* '<S72>/MemoryX' */
  uint8_T icLoad_e;                    /* '<S132>/MemoryX' */
  DW_ControllerFSFB_DroneRS_Com_T ControllerFSFB;/* '<S1>/ControllerFSFB' */
} DW_DroneRS_Compensator_DroneR_T;

/* Block signals (auto storage) */
typedef struct {
  B_DroneRS_Compensator_DroneRS_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
} B_DroneRS_Compensator_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  DW_DroneRS_Compensator_DroneR_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
} DW_DroneRS_Compensator_T;

/* Parameters for system: '<S1>/ControllerFSFB' */
struct P_ControllerFSFB_DroneRS_Comp_T_ {
  real_T takeoff_Gain_Gain;            /* Expression: controlParams.takeoff_Gain
                                        * Referenced by: '<S8>/takeoff_Gain'
                                        */
  real_T dz_ref_Value;                 /* Expression: 0
                                        * Referenced by: '<S6>/dz_ref'
                                        */
  real_T velocitiesPos_ref_Value[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<S6>/velocitiesPos_ref'
                                        */
  real_T velocitiesRot_ref_Value[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<S6>/velocitiesRot_ref'
                                        */
  real_T TorquetotalThrustToThrustperMot[16];/* Expression: controlParams.Q2Ts
                                              * Referenced by: '<S7>/TorquetotalThrustToThrustperMotor'
                                              */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S10>/Constant'
                                        */
  real_T SaturationThrust_UpperSat;    /* Expression: controlParams.totalThrust_maxRelative*controlParams.motorsThrust_i_UpperLimit*4
                                        * Referenced by: '<S8>/SaturationThrust'
                                        */
  real_T SaturationThrust_LowerSat;    /* Expression: -(controlParams.totalThrust_maxRelative*controlParams.motorsThrust_i_UpperLimit*4)
                                        * Referenced by: '<S8>/SaturationThrust'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: -0.0118
                                        * Referenced by: '<S7>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: -0.3235
                                        * Referenced by: '<S7>/Saturation2'
                                        */
  real_T MotorsRotationDirection_Gain[4];/* Expression: [-1,1,-1,1]
                                          * Referenced by: '<S9>/MotorsRotationDirection'
                                          */
};

/* Parameters for system: '<Root>/DroneRS_Compensator' */
struct P_DroneRS_Compensator_DroneRS_T_ {
  real_T DiscreteDerivative_ICPrevScaled;/* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                          * Referenced by: '<S120>/UD'
                                          */
  real_T checkPosavailable_const;      /* Mask Parameter: checkPosavailable_const
                                        * Referenced by: '<S181>/Constant'
                                        */
  real_T CompareToConstant_const;      /* Mask Parameter: CompareToConstant_const
                                        * Referenced by: '<S185>/Constant'
                                        */
  real_T outlierBelowFloor_const;      /* Mask Parameter: outlierBelowFloor_const
                                        * Referenced by: '<S19>/Constant'
                                        */
  real_T FIRaccelero_InitialStates;    /* Expression: 0
                                        * Referenced by: '<S14>/FIRaccelero'
                                        */
  real_T FIRaccelero_Coefficients[6];  /* Expression: controlParams.filter_accelero.Coefficients
                                        * Referenced by: '<S14>/FIRaccelero'
                                        */
  real_T IIRgyroz_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S14>/IIRgyroz'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S70>/Delay'
                                        */
  real_T KalmanGainM_Value[4];         /* Expression: pInitialization.M
                                        * Referenced by: '<S135>/KalmanGainM'
                                        */
  real_T IIRgyroz_InitialStates_c;     /* Expression: 0
                                        * Referenced by: '<S73>/IIRgyroz'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S120>/TSamp'
                                        */
  real_T invertzaxisGain_Gain;         /* Expression: -1
                                        * Referenced by: '<S11>/invertzaxisGain'
                                        */
  real_T SaturationSonar_LowerSat;     /* Expression: -inf
                                        * Referenced by: '<S16>/SaturationSonar'
                                        */
  real_T Delay2_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S11>/Delay2'
                                        */
  real_T IIRprs_InitialStates;         /* Expression: 0
                                        * Referenced by: '<S16>/IIRprs'
                                        */
  real_T IIRsonar_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S16>/IIRsonar'
                                        */
  real_T KalmanGainM_Value_h[2];       /* Expression: pInitialization.M
                                        * Referenced by: '<S20>/KalmanGainM'
                                        */
  real_T gravity_Value[3];             /* Expression: [0 0 quad.g]
                                        * Referenced by: '<S11>/gravity'
                                        */
  real_T C_Value[2];                   /* Expression: pInitialization.C
                                        * Referenced by: '<S15>/C'
                                        */
  real_T D_Value;                      /* Expression: pInitialization.D
                                        * Referenced by: '<S15>/D'
                                        */
  real_T X0_Value[2];                  /* Expression: pInitialization.X0
                                        * Referenced by: '<S15>/X0'
                                        */
  real_T Delay_InitialCondition_n;     /* Expression: 0
                                        * Referenced by: '<S69>/Delay'
                                        */
  real_T KalmanGainM_Value_f[4];       /* Expression: pInitialization.M
                                        * Referenced by: '<S75>/KalmanGainM'
                                        */
  real_T gravity_Value_g[3];           /* Expression: [0 0 -quad.g]
                                        * Referenced by: '<S71>/gravity'
                                        */
  real_T gainaccinput_Gain;            /* Expression: 0.2
                                        * Referenced by: '<S71>/gainaccinput'
                                        */
  real_T C_Value_d[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S72>/C'
                                        */
  real_T D_Value_f[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S72>/D'
                                        */
  real_T X0_Value_k[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S72>/X0'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S3>/Delay1'
                                        */
  real_T C_Value_f[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S132>/C'
                                        */
  real_T D_Value_f0[4];                /* Expression: pInitialization.D
                                        * Referenced by: '<S132>/D'
                                        */
  real_T X0_Value_d[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S132>/X0'
                                        */
  real_T SimplyIntegrateVelocity_gainval;/* Computed Parameter: SimplyIntegrateVelocity_gainval
                                          * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_IC;   /* Expression: 0
                                        * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                        */
  real_T SimplyIntegrateVelocity_UpperSa;/* Expression: 2
                                          * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_LowerSa;/* Expression: -2
                                          * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                          */
  real_T UseIPPosSwitch_Threshold;     /* Expression: 0
                                        * Referenced by: '<S70>/UseIPPosSwitch'
                                        */
  real_T A_Value[4];                   /* Expression: pInitialization.A
                                        * Referenced by: '<S15>/A'
                                        */
  real_T B_Value[2];                   /* Expression: pInitialization.B
                                        * Referenced by: '<S15>/B'
                                        */
  real_T KalmanGainL_Value[2];         /* Expression: pInitialization.L
                                        * Referenced by: '<S20>/KalmanGainL'
                                        */
  real_T A_Value_d[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S72>/A'
                                        */
  real_T B_Value_c[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S72>/B'
                                        */
  real_T KalmanGainL_Value_f[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S75>/KalmanGainL'
                                        */
  real_T A_Value_dj[4];                /* Expression: pInitialization.A
                                        * Referenced by: '<S132>/A'
                                        */
  real_T B_Value_k[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S132>/B'
                                        */
  real_T KalmanGainL_Value_j[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S135>/KalmanGainL'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S70>/Delay'
                                        */
  uint32_T Delay2_DelayLength;         /* Computed Parameter: Delay2_DelayLength
                                        * Referenced by: '<S11>/Delay2'
                                        */
  uint32_T MemoryX_DelayLength;        /* Computed Parameter: MemoryX_DelayLength
                                        * Referenced by: '<S15>/MemoryX'
                                        */
  uint32_T Delay_DelayLength_g;        /* Computed Parameter: Delay_DelayLength_g
                                        * Referenced by: '<S69>/Delay'
                                        */
  uint32_T MemoryX_DelayLength_g;      /* Computed Parameter: MemoryX_DelayLength_g
                                        * Referenced by: '<S72>/MemoryX'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S3>/Delay1'
                                        */
  uint32_T MemoryX_DelayLength_m;      /* Computed Parameter: MemoryX_DelayLength_m
                                        * Referenced by: '<S132>/MemoryX'
                                        */
  P_ControllerFSFB_DroneRS_Comp_T ControllerFSFB;/* '<S1>/ControllerFSFB' */
};

/* Parameters (auto storage) */
struct P_DroneRS_Compensator_T_ {
  struct_nVjCgugzLFJzCZr6yyeDeH quadEDT;/* Variable: quadEDT
                                         * Referenced by:
                                         *   '<S11>/prsToAlt_Gain'
                                         *   '<S14>/inversesIMU_Gain'
                                         *   '<S9>/W2ToMotorsCmd_Gain'
                                         *   '<S16>/SaturationSonar'
                                         *   '<S69>/opticalFlowToVelocity_Gain'
                                         *   '<S67>/Constant'
                                         */
  struct_pP0yJPvqYhejK9gHgcbWI quad;   /* Variable: quad
                                        * Referenced by:
                                        *   '<S8>/HoverThrustLinearizationPoint'
                                        *   '<S9>/ThrustToW2_Gain'
                                        */
  struct_eTOByJ6BrrCe8gZfBpKFUD altEstim;/* Variable: altEstim
                                          * Referenced by:
                                          *   '<S11>/Bias'
                                          *   '<S11>/Bias1'
                                          *   '<S14>/IIRgyroz'
                                          *   '<S16>/IIRprs'
                                          *   '<S16>/IIRsonar'
                                          *   '<S65>/Constant'
                                          *   '<S66>/Constant'
                                          *   '<S68>/Constant'
                                          *   '<S73>/IIRgyroz'
                                          */
  struct_rM3FFntOU5Aaym8djgtmlC ofhandle;/* Variable: ofhandle
                                          * Referenced by:
                                          *   '<S121>/Constant'
                                          *   '<S122>/Constant'
                                          *   '<S123>/Constant'
                                          *   '<S124>/Constant'
                                          *   '<S125>/Constant'
                                          *   '<S126>/Constant'
                                          *   '<S127>/Constant'
                                          *   '<S128>/Constant'
                                          *   '<S129>/Constant'
                                          *   '<S130>/Constant'
                                          *   '<S131>/Constant'
                                          */
  struct_YkbJnRR8M5ye4XtO88GdQC vishandle;/* Variable: vishandle
                                           * Referenced by:
                                           *   '<S182>/Constant'
                                           *   '<S183>/Constant'
                                           *   '<S184>/Constant'
                                           */
  real_T K_poleplace[48];              /* Variable: K_poleplace
                                        * Referenced by: '<S4>/FSFBMatrix_pp'
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
 * '<S2>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB'
 * '<S3>'   : 'sim_quadrotor/DroneRS_Compensator/Estimator'
 * '<S4>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/FullstateController'
 * '<S5>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/SysteminputConverter'
 * '<S6>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/statesReferences'
 * '<S7>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/SysteminputConverter/ControlMixerRS'
 * '<S8>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/SysteminputConverter/Takeoffphase_Thrustadjustment'
 * '<S9>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/SysteminputConverter/Thrust2Motorcmd'
 * '<S10>'  : 'sim_quadrotor/DroneRS_Compensator/ControllerFSFB/statesReferences/Compare To Zero'
 * '<S11>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude'
 * '<S12>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAttitude'
 * '<S13>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition'
 * '<S14>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/SensorPreprocessing'
 * '<S15>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude'
 * '<S16>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling'
 * '<S17>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/RStoWorldinacc'
 * '<S18>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/WorldToRSinacc'
 * '<S19>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/outlierBelowFloor'
 * '<S20>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL'
 * '<S21>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculateYhat'
 * '<S22>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionA'
 * '<S23>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionB'
 * '<S24>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionC'
 * '<S25>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionD'
 * '<S26>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionG'
 * '<S27>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionH'
 * '<S28>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionN'
 * '<S29>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP'
 * '<S30>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP0'
 * '<S31>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionQ'
 * '<S32>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionR'
 * '<S33>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionReset'
 * '<S34>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX'
 * '<S35>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX0'
 * '<S36>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionu'
 * '<S37>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/MemoryP'
 * '<S38>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer'
 * '<S39>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ReducedQRN'
 * '<S40>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionP0'
 * '<S41>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionQ'
 * '<S42>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionR'
 * '<S43>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator'
 * '<S44>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkA'
 * '<S45>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkB'
 * '<S46>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkC'
 * '<S47>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkD'
 * '<S48>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkEnable'
 * '<S49>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkG'
 * '<S50>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkH'
 * '<S51>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkN'
 * '<S52>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkP0'
 * '<S53>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkQ'
 * '<S54>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkR'
 * '<S55>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkReset'
 * '<S56>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkX0'
 * '<S57>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checku'
 * '<S58>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checky'
 * '<S59>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionL'
 * '<S60>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionM'
 * '<S61>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionP'
 * '<S62>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionZ'
 * '<S63>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer/MeasurementUpdate'
 * '<S64>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator/Enabled Subsystem'
 * '<S65>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffprs'
 * '<S66>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffsonarflt'
 * '<S67>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierDist_min'
 * '<S68>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierJump'
 * '<S69>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity'
 * '<S70>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition'
 * '<S71>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld'
 * '<S72>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy'
 * '<S73>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling'
 * '<S74>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld/World2Body'
 * '<S75>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL'
 * '<S76>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculateYhat'
 * '<S77>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionA'
 * '<S78>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionB'
 * '<S79>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionC'
 * '<S80>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionD'
 * '<S81>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionG'
 * '<S82>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionH'
 * '<S83>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionN'
 * '<S84>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP'
 * '<S85>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP0'
 * '<S86>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionQ'
 * '<S87>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionR'
 * '<S88>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionReset'
 * '<S89>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX'
 * '<S90>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX0'
 * '<S91>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionu'
 * '<S92>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/MemoryP'
 * '<S93>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer'
 * '<S94>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ReducedQRN'
 * '<S95>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionP0'
 * '<S96>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionQ'
 * '<S97>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionR'
 * '<S98>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator'
 * '<S99>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkA'
 * '<S100>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkB'
 * '<S101>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkC'
 * '<S102>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkD'
 * '<S103>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkEnable'
 * '<S104>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkG'
 * '<S105>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkH'
 * '<S106>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkN'
 * '<S107>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkP0'
 * '<S108>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkQ'
 * '<S109>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkR'
 * '<S110>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkReset'
 * '<S111>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkX0'
 * '<S112>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checku'
 * '<S113>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checky'
 * '<S114>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionL'
 * '<S115>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionM'
 * '<S116>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionP'
 * '<S117>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionZ'
 * '<S118>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer/MeasurementUpdate'
 * '<S119>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator/Enabled Subsystem'
 * '<S120>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/DiscreteDerivative'
 * '<S121>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxdw1'
 * '<S122>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxdw2'
 * '<S123>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxp'
 * '<S124>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxp2'
 * '<S125>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxq'
 * '<S126>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxq2'
 * '<S127>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw1'
 * '<S128>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw2'
 * '<S129>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw3'
 * '<S130>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw4'
 * '<S131>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/minHeightforOF'
 * '<S132>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy'
 * '<S133>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling'
 * '<S134>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/RStoWorld'
 * '<S135>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL'
 * '<S136>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculateYhat'
 * '<S137>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionA'
 * '<S138>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionB'
 * '<S139>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionC'
 * '<S140>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionD'
 * '<S141>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionG'
 * '<S142>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionH'
 * '<S143>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionN'
 * '<S144>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP'
 * '<S145>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP0'
 * '<S146>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionQ'
 * '<S147>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionR'
 * '<S148>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionReset'
 * '<S149>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX'
 * '<S150>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX0'
 * '<S151>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionu'
 * '<S152>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/MemoryP'
 * '<S153>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer'
 * '<S154>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ReducedQRN'
 * '<S155>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionP0'
 * '<S156>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionQ'
 * '<S157>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionR'
 * '<S158>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator'
 * '<S159>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkA'
 * '<S160>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkB'
 * '<S161>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkC'
 * '<S162>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkD'
 * '<S163>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkEnable'
 * '<S164>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkG'
 * '<S165>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkH'
 * '<S166>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkN'
 * '<S167>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkP0'
 * '<S168>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkQ'
 * '<S169>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkR'
 * '<S170>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkReset'
 * '<S171>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkX0'
 * '<S172>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checku'
 * '<S173>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checky'
 * '<S174>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionL'
 * '<S175>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionM'
 * '<S176>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionP'
 * '<S177>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionZ'
 * '<S178>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer/MeasurementUpdate'
 * '<S179>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator/Enabled Subsystem'
 * '<S180>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs'
 * '<S181>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/checkPosavailable'
 * '<S182>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxp3'
 * '<S183>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxq3'
 * '<S184>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/planarjumpsVISPOS'
 * '<S185>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/SensorPreprocessing/Compare To Constant'
 */
#endif                                 /* RTW_HEADER_DroneRS_Compensator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
