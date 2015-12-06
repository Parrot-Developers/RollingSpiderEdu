/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Drone_Compensator.h
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

#ifndef RTW_HEADER_Drone_Compensator_h_
#define RTW_HEADER_Drone_Compensator_h_
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <string.h>
#ifndef Drone_Compensator_COMMON_INCLUDES_
# define Drone_Compensator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_logging.h"
#endif                                 /* Drone_Compensator_COMMON_INCLUDES_ */

#include "Drone_Compensator_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

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

/* Block signals for system '<S1>/ControllerPID' */
typedef struct {
  real_T pos_ref[3];                   /* '<S2>/pos_ref' */
  real_T orient_ref[3];                /* '<S2>/orient_ref' */
  real_T Switch_refAtt[2];             /* '<S2>/Switch_refAtt' */
  real_T Motordirections1[4];          /* '<S6>/Motordirections1' */
} B_ControllerPID_Drone_Compens_T;

/* Block states (auto storage) for system '<S1>/ControllerPID' */
typedef struct {
  real_T Delay_DSTATE[2];              /* '<S2>/Delay' */
  real_T DiscreteTimeIntegrator_DSTATE[2];/* '<S2>/Discrete-Time Integrator' */
  struct {
    void *LoggedData;
  } SCOPE_totalThrust_PWORK;           /* '<S2>/SCOPE_totalThrust' */
} DW_ControllerPID_Drone_Compen_T;

/* Block signals for system '<S92>/MeasurementUpdate' */
typedef struct {
  real_T Product3[2];                  /* '<S117>/Product3' */
} B_MeasurementUpdate_Drone_Com_T;

/* Block signals for system '<S68>/UseCurrentEstimator' */
typedef struct {
  real_T Add[2];                       /* '<S97>/Add' */
  real_T Product2[2];                  /* '<S118>/Product2' */
} B_UseCurrentEstimator_Drone_C_T;

/* Block signals for system '<Root>/Drone_Compensator' */
typedef struct {
  real_T posVIS_datin[4];              /* '<S1>/posVIS_datin' */
  real_T sensordataCalib_datin[7];     /* '<S1>/sensordataCalib_datin' */
  real_T sensordata_datin[8];          /* '<S1>/sensordata_datin' */
  real_T usePosVIS_flagin;             /* '<S1>/usePosVIS_flagin' */
  real_T opticalFlow_datin[3];         /* '<S1>/opticalFlow_datin' */
  real_T FIR_IMUaccel[3];              /* '<S10>/FIR_IMUaccel' */
  real_T Reshapexhat[2];               /* '<S11>/Reshapexhat' */
  real_T Reshapexhat_o[2];             /* '<S68>/Reshapexhat' */
  real_T UseIPPosSwitch[2];            /* '<S66>/UseIPPosSwitch' */
  real_T batteryStatus_datin[2];       /* '<S1>/batteryStatus_datin' */
  real_T orient_estimout[3];           /* '<S3>/EstimatorOrientation' */
  real_T dorient_estimout[3];          /* '<S3>/EstimatorOrientation' */
  real_T acc_RS[3];                    /* '<S7>/trafo_WorldToBody_trans' */
  real_T Product2[2];                  /* '<S60>/Product2' */
  real_T Product3[2];                  /* '<S59>/Product3' */
  boolean_T takeoff_flag;              /* '<S1>/takeoff_flag' */
  boolean_T controlModePosVSOrient_flagin;/* '<S1>/controlModePosVSOrient_flagin' */
  B_UseCurrentEstimator_Drone_C_T UseCurrentEstimator_g;/* '<S131>/UseCurrentEstimator' */
  B_MeasurementUpdate_Drone_Com_T MeasurementUpdate_o;/* '<S152>/MeasurementUpdate' */
  B_UseCurrentEstimator_Drone_C_T UseCurrentEstimator_b;/* '<S68>/UseCurrentEstimator' */
  B_MeasurementUpdate_Drone_Com_T MeasurementUpdate_h;/* '<S92>/MeasurementUpdate' */
  B_ControllerPID_Drone_Compens_T ControllerPID;/* '<S1>/ControllerPID' */
} B_Drone_Compensator_Drone_Com_T;

/* Block states (auto storage) for system '<Root>/Drone_Compensator' */
typedef struct {
  real_T FIR_IMUaccel_states[15];      /* '<S10>/FIR_IMUaccel' */
  real_T IIR_IMUgyro_r_states[5];      /* '<S10>/IIR_IMUgyro_r' */
  real_T Delay_DSTATE[2];              /* '<S66>/Delay' */
  real_T IIRgyroz_states[10];          /* '<S69>/IIRgyroz' */
  real_T UD_DSTATE[2];                 /* '<S119>/UD' */
  real_T Delay_DSTATE_b[2];            /* '<S65>/Delay' */
  real_T Delay2_DSTATE;                /* '<S7>/Delay2' */
  real_T pressureFilter_IIR_states[5]; /* '<S12>/pressureFilter_IIR' */
  real_T soonarFilter_IIR_states[5];   /* '<S12>/soonarFilter_IIR' */
  real_T MemoryX_DSTATE[2];            /* '<S11>/MemoryX' */
  real_T MemoryX_DSTATE_g[2];          /* '<S68>/MemoryX' */
  real_T Delay1_DSTATE[2];             /* '<S3>/Delay1' */
  real_T MemoryX_DSTATE_m[2];          /* '<S131>/MemoryX' */
  real_T SimplyIntegrateVelocity_DSTATE[2];/* '<S66>/SimplyIntegrateVelocity' */
  real_T IIRgyroz_tmp[2];              /* '<S69>/IIRgyroz' */
  real_T yaw_cur;                      /* '<S3>/EstimatorOrientation' */
  real_T pitch_cur;                    /* '<S3>/EstimatorOrientation' */
  real_T roll_cur;                     /* '<S3>/EstimatorOrientation' */
  struct {
    void *LoggedData;
  } SCOPE_deuler_PWORK;                /* '<S3>/SCOPE_deuler' */

  struct {
    void *LoggedData;
  } SCOPE_orientation_Euler_estim_P;   /* '<S3>/SCOPE_orientation_Euler_estim' */

  struct {
    void *LoggedData;
  } SCOPE_altitude_Kalmanestimate_P;   /* '<S7>/SCOPE_altitude_Kalmanestimate' */

  struct {
    void *LoggedData;
  } SCOPE_altitude_pressure_PWORK;     /* '<S7>/SCOPE_altitude_pressure' */

  struct {
    void *LoggedData;
  } SCOPE_dxy_PWORK;                   /* '<S65>/SCOPE_dxy' */

  struct {
    void *LoggedData;
  } SCOPE_enableKFdxyupdate_PWORK;     /* '<S65>/SCOPE_enableKFdxyupdate' */

  int32_T FIR_IMUaccel_circBuf;        /* '<S10>/FIR_IMUaccel' */
  int8_T SimplyIntegrateVelocity_PrevRes;/* '<S66>/SimplyIntegrateVelocity' */
  uint8_T icLoad;                      /* '<S11>/MemoryX' */
  uint8_T icLoad_l;                    /* '<S68>/MemoryX' */
  uint8_T icLoad_j;                    /* '<S131>/MemoryX' */
  DW_ControllerPID_Drone_Compen_T ControllerPID;/* '<S1>/ControllerPID' */
} DW_Drone_Compensator_Drone_Co_T;

/* Block signals (auto storage) */
typedef struct {
  B_Drone_Compensator_Drone_Com_T Drone_Compensator_d;/* '<Root>/Drone_Compensator' */
} B_Drone_Compensator_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  DW_Drone_Compensator_Drone_Co_T Drone_Compensator_d;/* '<Root>/Drone_Compensator' */
} DW_Drone_Compensator_T;

/* Parameters for system: '<S1>/ControllerPID' */
struct P_ControllerPID_Drone_Compens_T_ {
  real_T D_z_Gain;                     /* Expression: 0.3
                                        * Referenced by: '<S2>/D_z'
                                        */
  real_T P_z_Gain;                     /* Expression: 0.8
                                        * Referenced by: '<S2>/P_z'
                                        */
  real_T D_xy_Gain[2];                 /* Expression: [0.1, -0.1]
                                        * Referenced by: '<S2>/D_xy'
                                        */
  real_T P_xy_Gain[2];                 /* Expression: [-0.24,0.24]
                                        * Referenced by: '<S2>/P_xy'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S2>/Delay'
                                        */
  real_T antiWU_Gain_Gain;             /* Expression: 0.001
                                        * Referenced by: '<S2>/antiWU_Gain'
                                        */
  real_T P_yaw_Gain;                   /* Expression: 0.004
                                        * Referenced by: '<S2>/P_yaw'
                                        */
  real_T D_yaw_Gain;                   /* Expression: 0.3*0.004
                                        * Referenced by: '<S2>/D_yaw'
                                        */
  real_T P_pr_Gain[2];                 /* Expression: [0.013;0.02]
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
  real_T I_pr_Gain;                    /* Expression: 0.01
                                        * Referenced by: '<S2>/I_pr'
                                        */
  real_T D_pr_Gain[2];                 /* Expression: [0.002;0.003]
                                        * Referenced by: '<S2>/D_pr'
                                        */
  real_T Saturation5_LowerSat;         /* Expression: 10
                                        * Referenced by: '<S6>/Saturation5'
                                        */
  real_T Motordirections1_Gain[4];     /* Expression: [1 -1 1 -1]
                                        * Referenced by: '<S6>/Motordirections1'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S2>/Delay'
                                        */
};

/* Parameters for system: '<Root>/Drone_Compensator' */
struct P_Drone_Compensator_Drone_Com_T_ {
  real_T DiscreteDerivative_ICPrevScaled;/* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                          * Referenced by: '<S119>/UD'
                                          */
  real_T checkifPosavailable_const;    /* Mask Parameter: checkifPosavailable_const
                                        * Referenced by: '<S180>/Constant'
                                        */
  real_T maxp3_const;                  /* Mask Parameter: maxp3_const
                                        * Referenced by: '<S181>/Constant'
                                        */
  real_T maxq3_const;                  /* Mask Parameter: maxq3_const
                                        * Referenced by: '<S182>/Constant'
                                        */
  real_T planarjumpsVISPOS_const;      /* Mask Parameter: planarjumpsVISPOS_const
                                        * Referenced by: '<S183>/Constant'
                                        */
  real_T maxp_const;                   /* Mask Parameter: maxp_const
                                        * Referenced by: '<S122>/Constant'
                                        */
  real_T maxq_const;                   /* Mask Parameter: maxq_const
                                        * Referenced by: '<S124>/Constant'
                                        */
  real_T maxw1_const;                  /* Mask Parameter: maxw1_const
                                        * Referenced by: '<S126>/Constant'
                                        */
  real_T maxw2_const;                  /* Mask Parameter: maxw2_const
                                        * Referenced by: '<S127>/Constant'
                                        */
  real_T maxdw1_const;                 /* Mask Parameter: maxdw1_const
                                        * Referenced by: '<S120>/Constant'
                                        */
  real_T maxdw2_const;                 /* Mask Parameter: maxdw2_const
                                        * Referenced by: '<S121>/Constant'
                                        */
  real_T maxp2_const;                  /* Mask Parameter: maxp2_const
                                        * Referenced by: '<S123>/Constant'
                                        */
  real_T maxq2_const;                  /* Mask Parameter: maxq2_const
                                        * Referenced by: '<S125>/Constant'
                                        */
  real_T maxw3_const;                  /* Mask Parameter: maxw3_const
                                        * Referenced by: '<S128>/Constant'
                                        */
  real_T maxw4_const;                  /* Mask Parameter: maxw4_const
                                        * Referenced by: '<S129>/Constant'
                                        */
  real_T outlierJump_const;            /* Mask Parameter: outlierJump_const
                                        * Referenced by: '<S64>/Constant'
                                        */
  real_T currentEstimateVeryOffFromPress;/* Mask Parameter: currentEstimateVeryOffFromPress
                                          * Referenced by: '<S62>/Constant'
                                          */
  real_T currentStateVeryOffsonarflt_con;/* Mask Parameter: currentStateVeryOffsonarflt_con
                                          * Referenced by: '<S63>/Constant'
                                          */
  real_T outlierBelowFloor_const;      /* Mask Parameter: outlierBelowFloor_const
                                        * Referenced by: '<S13>/Constant'
                                        */
  real_T minHeightforOF_const;         /* Mask Parameter: minHeightforOF_const
                                        * Referenced by: '<S130>/Constant'
                                        */
  real_T donotuseaccifopticalflowneverav;/* Mask Parameter: donotuseaccifopticalflowneverav
                                          * Referenced by: '<S71>/Constant'
                                          */
  real_T donotuseaccifopticalflownever_g;/* Mask Parameter: donotuseaccifopticalflownever_g
                                          * Referenced by: '<S72>/Constant'
                                          */
  real_T DeactivateAccelerationIfOFisnot;/* Mask Parameter: DeactivateAccelerationIfOFisnot
                                          * Referenced by: '<S70>/Constant'
                                          */
  real_T Assumingthatcalibwasdonelevel_B[6];/* Expression: [0 0 +quad.g 0 0 0]
                                             * Referenced by: '<S10>/Assuming that calib was done level!'
                                             */
  real_T FIR_IMUaccel_InitialStates;   /* Expression: 0
                                        * Referenced by: '<S10>/FIR_IMUaccel'
                                        */
  real_T FIR_IMUaccel_Coefficients[6]; /* Expression: estimParams.IMU.filter_accel.Coefficients
                                        * Referenced by: '<S10>/FIR_IMUaccel'
                                        */
  real_T IIR_IMUgyro_r_NumCoef[6];     /* Expression: estimParams.IMU.filter_gyro_r_b
                                        * Referenced by: '<S10>/IIR_IMUgyro_r'
                                        */
  real_T IIR_IMUgyro_r_DenCoef[6];     /* Expression: estimParams.IMU.filter_gyro_r_a
                                        * Referenced by: '<S10>/IIR_IMUgyro_r'
                                        */
  real_T IIR_IMUgyro_r_InitialStates;  /* Expression: 0
                                        * Referenced by: '<S10>/IIR_IMUgyro_r'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S66>/Delay'
                                        */
  real_T KalmanGainM_Value[4];         /* Expression: pInitialization.M
                                        * Referenced by: '<S134>/KalmanGainM'
                                        */
  real_T IIRgyroz_NumCoef[6];          /* Expression: estimParams.IMU.filter_gyro_r_b
                                        * Referenced by: '<S69>/IIRgyroz'
                                        */
  real_T IIRgyroz_DenCoef[6];          /* Expression: estimParams.IMU.filter_gyro_r_b
                                        * Referenced by: '<S69>/IIRgyroz'
                                        */
  real_T IIRgyroz_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S69>/IIRgyroz'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S119>/TSamp'
                                        */
  real_T Delay_InitialCondition_h;     /* Expression: 0
                                        * Referenced by: '<S65>/Delay'
                                        */
  real_T opticalFlowToVelocity_gain_Gain;/* Expression: estimParams.pos.opticalFlowToVelocity_gain
                                          * Referenced by: '<S65>/opticalFlowToVelocity_gain'
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
  real_T pressureFilter_IIR_NumCoef[6];/* Expression: estimParams.alt.filter_prs_b
                                        * Referenced by: '<S12>/pressureFilter_IIR'
                                        */
  real_T pressureFilter_IIR_DenCoef[6];/* Expression: estimParams.alt.filter_prs_a
                                        * Referenced by: '<S12>/pressureFilter_IIR'
                                        */
  real_T pressureFilter_IIR_InitialState;/* Expression: 0
                                          * Referenced by: '<S12>/pressureFilter_IIR'
                                          */
  real_T soonarFilter_IIR_NumCoef[6];  /* Expression: estimParams.alt.filter_sonar_b
                                        * Referenced by: '<S12>/soonarFilter_IIR'
                                        */
  real_T soonarFilter_IIR_DenCoef[6];  /* Expression: estimParams.alt.filter_sonar_a
                                        * Referenced by: '<S12>/soonarFilter_IIR'
                                        */
  real_T soonarFilter_IIR_InitialStates;/* Expression: 0
                                         * Referenced by: '<S12>/soonarFilter_IIR'
                                         */
  real_T KalmanGainM_Value_p[2];       /* Expression: pInitialization.M
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
  real_T KalmanGainM_Value_n[4];       /* Expression: pInitialization.M
                                        * Referenced by: '<S74>/KalmanGainM'
                                        */
  real_T gravity_Value_b[3];           /* Expression: [0 0 -quad.g]
                                        * Referenced by: '<S67>/gravity'
                                        */
  real_T gainaccinput_Gain;            /* Expression: estimParams.pos.accelerationInput_gain
                                        * Referenced by: '<S67>/gainaccinput'
                                        */
  real_T C_Value_j[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S68>/C'
                                        */
  real_T D_Value_b[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S68>/D'
                                        */
  real_T X0_Value_j[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S68>/X0'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S3>/Delay1'
                                        */
  real_T C_Value_f[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S131>/C'
                                        */
  real_T D_Value_o[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S131>/D'
                                        */
  real_T X0_Value_k[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S131>/X0'
                                        */
  real_T SimplyIntegrateVelocity_gainval;/* Computed Parameter: SimplyIntegrateVelocity_gainval
                                          * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_IC;   /* Expression: 0
                                        * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                        */
  real_T SimplyIntegrateVelocity_UpperSa;/* Expression: estimParams.pos.veloIntegrator_max
                                          * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_LowerSa;/* Expression: -estimParams.pos.veloIntegrator_max
                                          * Referenced by: '<S66>/SimplyIntegrateVelocity'
                                          */
  real_T UseIPPosSwitch_Threshold;     /* Expression: 0
                                        * Referenced by: '<S66>/UseIPPosSwitch'
                                        */
  real_T Bias_Bias;                    /* Expression: estimParams.alt.deltaSonarToCurrent_max
                                        * Referenced by: '<S7>/Bias'
                                        */
  real_T Bias1_Bias;                   /* Expression: -estimParams.alt.deltaSonarToCurrent_max
                                        * Referenced by: '<S7>/Bias1'
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
  real_T A_Value_m[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S68>/A'
                                        */
  real_T B_Value_b[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S68>/B'
                                        */
  real_T KalmanGainL_Value_p[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S74>/KalmanGainL'
                                        */
  real_T A_Value_g[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S131>/A'
                                        */
  real_T B_Value_a[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S131>/B'
                                        */
  real_T KalmanGainL_Value_o[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S134>/KalmanGainL'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S66>/Delay'
                                        */
  uint32_T Delay_DelayLength_i;        /* Computed Parameter: Delay_DelayLength_i
                                        * Referenced by: '<S65>/Delay'
                                        */
  uint32_T Delay2_DelayLength;         /* Computed Parameter: Delay2_DelayLength
                                        * Referenced by: '<S7>/Delay2'
                                        */
  uint32_T MemoryX_DelayLength;        /* Computed Parameter: MemoryX_DelayLength
                                        * Referenced by: '<S11>/MemoryX'
                                        */
  uint32_T MemoryX_DelayLength_e;      /* Computed Parameter: MemoryX_DelayLength_e
                                        * Referenced by: '<S68>/MemoryX'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S3>/Delay1'
                                        */
  uint32_T MemoryX_DelayLength_a;      /* Computed Parameter: MemoryX_DelayLength_a
                                        * Referenced by: '<S131>/MemoryX'
                                        */
  P_ControllerPID_Drone_Compens_T ControllerPID;/* '<S1>/ControllerPID' */
};

/* Parameters (auto storage) */
struct P_Drone_Compensator_T_ {
  struct_fgpOXIBQXBGEsNX5uiZ1j quadEDT;/* Variable: quadEDT
                                        * Referenced by:
                                        *   '<S3>/sampleTime'
                                        *   '<S6>/thrustToMotorcommand'
                                        *   '<S6>/Saturation5'
                                        *   '<S7>/prsToAlt_gain'
                                        *   '<S10>/inverseIMU_gain'
                                        *   '<S12>/SaturationSonar'
                                        *   '<S184>/Constant'
                                        *   '<S61>/Constant'
                                        */
  struct_pP0yJPvqYhejK9gHgcbWI quad;   /* Variable: quad
                                        * Referenced by:
                                        *   '<S2>/w0'
                                        *   '<S3>/sampleTime1'
                                        */
  struct_SNZkZ6rc9vhdEg27lKQdHF controlHelperParams;/* Variable: controlHelperParams
                                                     * Referenced by:
                                                     *   '<S2>/takeoff_gain'
                                                     *   '<S2>/SaturationThrust'
                                                     *   '<S4>/TorquetotalThrustToThrustperMotor'
                                                     */
  P_Drone_Compensator_Drone_Com_T Drone_Compensator_d;/* '<Root>/Drone_Compensator' */
};

/* Real-time Model Data Structure */
struct tag_RTM_Drone_Compensator_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * ModelData:
   * The following substructure contains information regarding
   * the data used in the model.
   */
  struct {
    B_Drone_Compensator_T *blockIO;
    P_Drone_Compensator_T *defaultParam;
    DW_Drone_Compensator_T *dwork;
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
extern void Drone_Compensator_initialize(RT_MODEL_Drone_Compensator_T *const
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
  *Drone_Compensator_Y_takeoff_flagout);
extern void Drone_Compensator_step(RT_MODEL_Drone_Compensator_T *const
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
  *Drone_Compensator_Y_takeoff_flagout);

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
 * hilite_system('sim_quadrotor/Drone_Compensator')    - opens subsystem sim_quadrotor/Drone_Compensator
 * hilite_system('sim_quadrotor/Drone_Compensator/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'sim_quadrotor'
 * '<S1>'   : 'sim_quadrotor/Drone_Compensator'
 * '<S2>'   : 'sim_quadrotor/Drone_Compensator/ControllerPID'
 * '<S3>'   : 'sim_quadrotor/Drone_Compensator/Estimator'
 * '<S4>'   : 'sim_quadrotor/Drone_Compensator/ControllerPID/ControlMixer'
 * '<S5>'   : 'sim_quadrotor/Drone_Compensator/ControllerPID/inverse rotation Function'
 * '<S6>'   : 'sim_quadrotor/Drone_Compensator/ControllerPID/thrustsToMotorCommands'
 * '<S7>'   : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude'
 * '<S8>'   : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorOrientation'
 * '<S9>'   : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition'
 * '<S10>'  : 'sim_quadrotor/Drone_Compensator/Estimator/SensorPreprocessing'
 * '<S11>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude'
 * '<S12>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling'
 * '<S13>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/outlierBelowFloor'
 * '<S14>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/trafo_BodytoWorld_trans'
 * '<S15>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/trafo_WorldToBody_trans'
 * '<S16>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL'
 * '<S17>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculateYhat'
 * '<S18>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionA'
 * '<S19>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionB'
 * '<S20>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionC'
 * '<S21>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionD'
 * '<S22>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionG'
 * '<S23>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionH'
 * '<S24>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionN'
 * '<S25>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP'
 * '<S26>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP0'
 * '<S27>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionQ'
 * '<S28>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionR'
 * '<S29>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionReset'
 * '<S30>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX'
 * '<S31>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX0'
 * '<S32>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionu'
 * '<S33>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/MemoryP'
 * '<S34>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer'
 * '<S35>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ReducedQRN'
 * '<S36>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionP0'
 * '<S37>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionQ'
 * '<S38>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionR'
 * '<S39>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator'
 * '<S40>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkA'
 * '<S41>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkB'
 * '<S42>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkC'
 * '<S43>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkD'
 * '<S44>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkEnable'
 * '<S45>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkG'
 * '<S46>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkH'
 * '<S47>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkN'
 * '<S48>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkP0'
 * '<S49>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkQ'
 * '<S50>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkR'
 * '<S51>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkReset'
 * '<S52>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkX0'
 * '<S53>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checku'
 * '<S54>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checky'
 * '<S55>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionL'
 * '<S56>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionM'
 * '<S57>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionP'
 * '<S58>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionZ'
 * '<S59>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer/MeasurementUpdate'
 * '<S60>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator/Enabled Subsystem'
 * '<S61>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/check for min altitude'
 * '<S62>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentEstimateVeryOffFromPressure'
 * '<S63>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffsonarflt'
 * '<S64>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierJump'
 * '<S65>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity'
 * '<S66>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition'
 * '<S67>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling'
 * '<S68>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy'
 * '<S69>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling'
 * '<S70>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/Deactivate Acceleration If OF is not used due to low altitude'
 * '<S71>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/do not use acc if optical flow never available (Note OF@60Hz but ZOH to 200!)'
 * '<S72>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/do not use acc if optical flow never available (Note OF@60Hz but ZOH to 200!)1'
 * '<S73>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/trafo_World2Body_trans'
 * '<S74>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL'
 * '<S75>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculateYhat'
 * '<S76>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionA'
 * '<S77>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionB'
 * '<S78>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionC'
 * '<S79>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionD'
 * '<S80>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionG'
 * '<S81>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionH'
 * '<S82>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionN'
 * '<S83>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP'
 * '<S84>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP0'
 * '<S85>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionQ'
 * '<S86>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionR'
 * '<S87>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionReset'
 * '<S88>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX'
 * '<S89>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX0'
 * '<S90>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionu'
 * '<S91>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/MemoryP'
 * '<S92>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer'
 * '<S93>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ReducedQRN'
 * '<S94>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionP0'
 * '<S95>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionQ'
 * '<S96>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionR'
 * '<S97>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator'
 * '<S98>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkA'
 * '<S99>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkB'
 * '<S100>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkC'
 * '<S101>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkD'
 * '<S102>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkEnable'
 * '<S103>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkG'
 * '<S104>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkH'
 * '<S105>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkN'
 * '<S106>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkP0'
 * '<S107>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkQ'
 * '<S108>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkR'
 * '<S109>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkReset'
 * '<S110>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkX0'
 * '<S111>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checku'
 * '<S112>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checky'
 * '<S113>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionL'
 * '<S114>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionM'
 * '<S115>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionP'
 * '<S116>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionZ'
 * '<S117>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer/MeasurementUpdate'
 * '<S118>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator/Enabled Subsystem'
 * '<S119>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/DiscreteDerivative'
 * '<S120>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxdw1'
 * '<S121>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxdw2'
 * '<S122>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxp'
 * '<S123>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxp2'
 * '<S124>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxq'
 * '<S125>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxq2'
 * '<S126>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw1'
 * '<S127>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw2'
 * '<S128>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw3'
 * '<S129>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw4'
 * '<S130>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/minHeightforOF'
 * '<S131>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy'
 * '<S132>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling'
 * '<S133>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/trafo_BodytoWorld_trans'
 * '<S134>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL'
 * '<S135>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculateYhat'
 * '<S136>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionA'
 * '<S137>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionB'
 * '<S138>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionC'
 * '<S139>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionD'
 * '<S140>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionG'
 * '<S141>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionH'
 * '<S142>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionN'
 * '<S143>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP'
 * '<S144>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP0'
 * '<S145>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionQ'
 * '<S146>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionR'
 * '<S147>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionReset'
 * '<S148>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX'
 * '<S149>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX0'
 * '<S150>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionu'
 * '<S151>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/MemoryP'
 * '<S152>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer'
 * '<S153>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ReducedQRN'
 * '<S154>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionP0'
 * '<S155>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionQ'
 * '<S156>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionR'
 * '<S157>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator'
 * '<S158>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkA'
 * '<S159>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkB'
 * '<S160>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkC'
 * '<S161>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkD'
 * '<S162>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkEnable'
 * '<S163>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkG'
 * '<S164>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkH'
 * '<S165>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkN'
 * '<S166>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkP0'
 * '<S167>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkQ'
 * '<S168>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkR'
 * '<S169>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkReset'
 * '<S170>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkX0'
 * '<S171>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checku'
 * '<S172>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checky'
 * '<S173>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionL'
 * '<S174>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionM'
 * '<S175>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionP'
 * '<S176>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionZ'
 * '<S177>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer/MeasurementUpdate'
 * '<S178>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator/Enabled Subsystem'
 * '<S179>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs'
 * '<S180>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/checkifPosavailable'
 * '<S181>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxp3'
 * '<S182>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxq3'
 * '<S183>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/planarjumpsVISPOS'
 * '<S184>' : 'sim_quadrotor/Drone_Compensator/Estimator/SensorPreprocessing/Check if valid visual position estimate available'
 */
#endif                                 /* RTW_HEADER_Drone_Compensator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
