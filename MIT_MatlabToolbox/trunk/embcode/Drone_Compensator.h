/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Drone_Compensator.h
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

/* Block signals for system '<S1>/ControllerPolePlace' */
typedef struct {
  real_T PosVSOrient_Switch[12];       /* '<S6>/PosVSOrient_Switch' */
  real_T W2ToMotorsCmd_Gain[4];        /* '<S9>/W2ToMotorsCmd_Gain' */
} B_ControllerPolePlace_Drone_C_T;

/* Block states (auto storage) for system '<S1>/ControllerPolePlace' */
typedef struct {
  struct {
    void *LoggedData;
  } SCOPE_totalthrust_PWORK;           /* '<S8>/SCOPE_totalthrust ' */
} DW_ControllerPolePlace_Drone__T;

/* Block signals for system '<S95>/MeasurementUpdate' */
typedef struct {
  real_T Product3[2];                  /* '<S120>/Product3' */
} B_MeasurementUpdate_Drone_Com_T;

/* Block signals for system '<S71>/UseCurrentEstimator' */
typedef struct {
  real_T Add[2];                       /* '<S100>/Add' */
  real_T Product2[2];                  /* '<S121>/Product2' */
} B_UseCurrentEstimator_Drone_C_T;

/* Block signals for system '<Root>/Drone_Compensator' */
typedef struct {
  real_T posVIS_datin[4];              /* '<S1>/posVIS_datin' */
  real_T sensordataCalib_datin[7];     /* '<S1>/sensordataCalib_datin' */
  real_T sensordata_datin[8];          /* '<S1>/sensordata_datin' */
  real_T usePosVIS_flagin;             /* '<S1>/usePosVIS_flagin' */
  real_T opticalFlow_datin[3];         /* '<S1>/opticalFlow_datin' */
  real_T FIR_IMUaccel[3];              /* '<S13>/FIR_IMUaccel' */
  real_T Reshapexhat[2];               /* '<S14>/Reshapexhat' */
  real_T Reshapexhat_o[2];             /* '<S71>/Reshapexhat' */
  real_T UseIPPosSwitch[2];            /* '<S69>/UseIPPosSwitch' */
  real_T batteryStatus_datin[2];       /* '<S1>/batteryStatus_datin' */
  real_T orient_estimout[3];           /* '<S3>/EstimatorOrientation' */
  real_T dorient_estimout[3];          /* '<S3>/EstimatorOrientation' */
  real_T acc_RS[3];                    /* '<S10>/trafo_WorldToBody_trans' */
  real_T Product2[2];                  /* '<S63>/Product2' */
  real_T Product3[2];                  /* '<S62>/Product3' */
  boolean_T takeoff_flag;              /* '<S1>/takeoff_flag' */
  boolean_T controlModePosVSOrient_flagin;/* '<S1>/controlModePosVSOrient_flagin' */
  B_UseCurrentEstimator_Drone_C_T UseCurrentEstimator_g;/* '<S134>/UseCurrentEstimator' */
  B_MeasurementUpdate_Drone_Com_T MeasurementUpdate_o;/* '<S155>/MeasurementUpdate' */
  B_UseCurrentEstimator_Drone_C_T UseCurrentEstimator_b;/* '<S71>/UseCurrentEstimator' */
  B_MeasurementUpdate_Drone_Com_T MeasurementUpdate_h;/* '<S95>/MeasurementUpdate' */
  B_ControllerPolePlace_Drone_C_T ControllerPolePlace;/* '<S1>/ControllerPolePlace' */
} B_Drone_Compensator_Drone_Com_T;

/* Block states (auto storage) for system '<Root>/Drone_Compensator' */
typedef struct {
  real_T FIR_IMUaccel_states[15];      /* '<S13>/FIR_IMUaccel' */
  real_T IIR_IMUgyro_r_states[5];      /* '<S13>/IIR_IMUgyro_r' */
  real_T Delay_DSTATE[2];              /* '<S69>/Delay' */
  real_T IIRgyroz_states[10];          /* '<S72>/IIRgyroz' */
  real_T UD_DSTATE[2];                 /* '<S122>/UD' */
  real_T Delay_DSTATE_b[2];            /* '<S68>/Delay' */
  real_T Delay2_DSTATE;                /* '<S10>/Delay2' */
  real_T pressureFilter_IIR_states[5]; /* '<S15>/pressureFilter_IIR' */
  real_T soonarFilter_IIR_states[5];   /* '<S15>/soonarFilter_IIR' */
  real_T MemoryX_DSTATE[2];            /* '<S14>/MemoryX' */
  real_T MemoryX_DSTATE_g[2];          /* '<S71>/MemoryX' */
  real_T Delay1_DSTATE[2];             /* '<S3>/Delay1' */
  real_T MemoryX_DSTATE_m[2];          /* '<S134>/MemoryX' */
  real_T SimplyIntegrateVelocity_DSTATE[2];/* '<S69>/SimplyIntegrateVelocity' */
  real_T IIRgyroz_tmp[2];              /* '<S72>/IIRgyroz' */
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
  } SCOPE_altitude_Kalmanestimate_P;   /* '<S10>/SCOPE_altitude_Kalmanestimate' */

  struct {
    void *LoggedData;
  } SCOPE_altitude_pressure_PWORK;     /* '<S10>/SCOPE_altitude_pressure' */

  struct {
    void *LoggedData;
  } SCOPE_dxy_PWORK;                   /* '<S68>/SCOPE_dxy' */

  struct {
    void *LoggedData;
  } SCOPE_enableKFdxyupdate_PWORK;     /* '<S68>/SCOPE_enableKFdxyupdate' */

  int32_T FIR_IMUaccel_circBuf;        /* '<S13>/FIR_IMUaccel' */
  int8_T SimplyIntegrateVelocity_PrevRes;/* '<S69>/SimplyIntegrateVelocity' */
  uint8_T icLoad;                      /* '<S14>/MemoryX' */
  uint8_T icLoad_l;                    /* '<S71>/MemoryX' */
  uint8_T icLoad_j;                    /* '<S134>/MemoryX' */
  DW_ControllerPolePlace_Drone__T ControllerPolePlace;/* '<S1>/ControllerPolePlace' */
} DW_Drone_Compensator_Drone_Co_T;

/* Block signals (auto storage) */
typedef struct {
  B_Drone_Compensator_Drone_Com_T Drone_Compensator_d;/* '<Root>/Drone_Compensator' */
} B_Drone_Compensator_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  DW_Drone_Compensator_Drone_Co_T Drone_Compensator_d;/* '<Root>/Drone_Compensator' */
} DW_Drone_Compensator_T;

/* Parameters for system: '<S1>/ControllerPolePlace' */
struct P_ControllerPolePlace_Drone_C_T_ {
  real_T dz_ref_Value;                 /* Expression: 0
                                        * Referenced by: '<S6>/dz_ref'
                                        */
  real_T velocitiesPos_ref_Value[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<S6>/velocitiesPos_ref'
                                        */
  real_T velocitiesRot_ref_Value[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<S6>/velocitiesRot_ref'
                                        */
  real_T MotorsRotationDirection_Gain[4];/* Expression: [-1,1,-1,1]
                                          * Referenced by: '<S9>/MotorsRotationDirection'
                                          */
};

/* Parameters for system: '<Root>/Drone_Compensator' */
struct P_Drone_Compensator_Drone_Com_T_ {
  real_T DiscreteDerivative_ICPrevScaled;/* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                          * Referenced by: '<S122>/UD'
                                          */
  real_T checkifPosavailable_const;    /* Mask Parameter: checkifPosavailable_const
                                        * Referenced by: '<S183>/Constant'
                                        */
  real_T maxp3_const;                  /* Mask Parameter: maxp3_const
                                        * Referenced by: '<S184>/Constant'
                                        */
  real_T maxq3_const;                  /* Mask Parameter: maxq3_const
                                        * Referenced by: '<S185>/Constant'
                                        */
  real_T planarjumpsVISPOS_const;      /* Mask Parameter: planarjumpsVISPOS_const
                                        * Referenced by: '<S186>/Constant'
                                        */
  real_T maxp_const;                   /* Mask Parameter: maxp_const
                                        * Referenced by: '<S125>/Constant'
                                        */
  real_T maxq_const;                   /* Mask Parameter: maxq_const
                                        * Referenced by: '<S127>/Constant'
                                        */
  real_T maxw1_const;                  /* Mask Parameter: maxw1_const
                                        * Referenced by: '<S129>/Constant'
                                        */
  real_T maxw2_const;                  /* Mask Parameter: maxw2_const
                                        * Referenced by: '<S130>/Constant'
                                        */
  real_T maxdw1_const;                 /* Mask Parameter: maxdw1_const
                                        * Referenced by: '<S123>/Constant'
                                        */
  real_T maxdw2_const;                 /* Mask Parameter: maxdw2_const
                                        * Referenced by: '<S124>/Constant'
                                        */
  real_T maxp2_const;                  /* Mask Parameter: maxp2_const
                                        * Referenced by: '<S126>/Constant'
                                        */
  real_T maxq2_const;                  /* Mask Parameter: maxq2_const
                                        * Referenced by: '<S128>/Constant'
                                        */
  real_T maxw3_const;                  /* Mask Parameter: maxw3_const
                                        * Referenced by: '<S131>/Constant'
                                        */
  real_T maxw4_const;                  /* Mask Parameter: maxw4_const
                                        * Referenced by: '<S132>/Constant'
                                        */
  real_T outlierJump_const;            /* Mask Parameter: outlierJump_const
                                        * Referenced by: '<S67>/Constant'
                                        */
  real_T currentEstimateVeryOffFromPress;/* Mask Parameter: currentEstimateVeryOffFromPress
                                          * Referenced by: '<S65>/Constant'
                                          */
  real_T currentStateVeryOffsonarflt_con;/* Mask Parameter: currentStateVeryOffsonarflt_con
                                          * Referenced by: '<S66>/Constant'
                                          */
  real_T outlierBelowFloor_const;      /* Mask Parameter: outlierBelowFloor_const
                                        * Referenced by: '<S16>/Constant'
                                        */
  real_T minHeightforOF_const;         /* Mask Parameter: minHeightforOF_const
                                        * Referenced by: '<S133>/Constant'
                                        */
  real_T donotuseaccifopticalflowneverav;/* Mask Parameter: donotuseaccifopticalflowneverav
                                          * Referenced by: '<S74>/Constant'
                                          */
  real_T donotuseaccifopticalflownever_g;/* Mask Parameter: donotuseaccifopticalflownever_g
                                          * Referenced by: '<S75>/Constant'
                                          */
  real_T DeactivateAccelerationIfOFisnot;/* Mask Parameter: DeactivateAccelerationIfOFisnot
                                          * Referenced by: '<S73>/Constant'
                                          */
  real_T Assumingthatcalibwasdonelevel_B[6];/* Expression: [0 0 +quad.g 0 0 0]
                                             * Referenced by: '<S13>/Assuming that calib was done level!'
                                             */
  real_T FIR_IMUaccel_InitialStates;   /* Expression: 0
                                        * Referenced by: '<S13>/FIR_IMUaccel'
                                        */
  real_T FIR_IMUaccel_Coefficients[6]; /* Expression: estimParams.IMU.filter_accel.Coefficients
                                        * Referenced by: '<S13>/FIR_IMUaccel'
                                        */
  real_T IIR_IMUgyro_r_NumCoef[6];     /* Expression: estimParams.IMU.filter_gyro_r_b
                                        * Referenced by: '<S13>/IIR_IMUgyro_r'
                                        */
  real_T IIR_IMUgyro_r_DenCoef[6];     /* Expression: estimParams.IMU.filter_gyro_r_a
                                        * Referenced by: '<S13>/IIR_IMUgyro_r'
                                        */
  real_T IIR_IMUgyro_r_InitialStates;  /* Expression: 0
                                        * Referenced by: '<S13>/IIR_IMUgyro_r'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S69>/Delay'
                                        */
  real_T KalmanGainM_Value[4];         /* Expression: pInitialization.M
                                        * Referenced by: '<S137>/KalmanGainM'
                                        */
  real_T IIRgyroz_NumCoef[6];          /* Expression: estimParams.IMU.filter_gyro_r_b
                                        * Referenced by: '<S72>/IIRgyroz'
                                        */
  real_T IIRgyroz_DenCoef[6];          /* Expression: estimParams.IMU.filter_gyro_r_b
                                        * Referenced by: '<S72>/IIRgyroz'
                                        */
  real_T IIRgyroz_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S72>/IIRgyroz'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S122>/TSamp'
                                        */
  real_T Delay_InitialCondition_h;     /* Expression: 0
                                        * Referenced by: '<S68>/Delay'
                                        */
  real_T opticalFlowToVelocity_gain_Gain;/* Expression: estimParams.pos.opticalFlowToVelocity_gain
                                          * Referenced by: '<S68>/opticalFlowToVelocity_gain'
                                          */
  real_T invertzaxisGain_Gain;         /* Expression: -1
                                        * Referenced by: '<S10>/invertzaxisGain'
                                        */
  real_T SaturationSonar_LowerSat;     /* Expression: -inf
                                        * Referenced by: '<S15>/SaturationSonar'
                                        */
  real_T Delay2_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S10>/Delay2'
                                        */
  real_T pressureFilter_IIR_NumCoef[6];/* Expression: estimParams.alt.filter_prs_b
                                        * Referenced by: '<S15>/pressureFilter_IIR'
                                        */
  real_T pressureFilter_IIR_DenCoef[6];/* Expression: estimParams.alt.filter_prs_a
                                        * Referenced by: '<S15>/pressureFilter_IIR'
                                        */
  real_T pressureFilter_IIR_InitialState;/* Expression: 0
                                          * Referenced by: '<S15>/pressureFilter_IIR'
                                          */
  real_T soonarFilter_IIR_NumCoef[6];  /* Expression: estimParams.alt.filter_sonar_b
                                        * Referenced by: '<S15>/soonarFilter_IIR'
                                        */
  real_T soonarFilter_IIR_DenCoef[6];  /* Expression: estimParams.alt.filter_sonar_a
                                        * Referenced by: '<S15>/soonarFilter_IIR'
                                        */
  real_T soonarFilter_IIR_InitialStates;/* Expression: 0
                                         * Referenced by: '<S15>/soonarFilter_IIR'
                                         */
  real_T KalmanGainM_Value_p[2];       /* Expression: pInitialization.M
                                        * Referenced by: '<S19>/KalmanGainM'
                                        */
  real_T gravity_Value[3];             /* Expression: [0 0 quad.g]
                                        * Referenced by: '<S10>/gravity'
                                        */
  real_T C_Value[2];                   /* Expression: pInitialization.C
                                        * Referenced by: '<S14>/C'
                                        */
  real_T D_Value;                      /* Expression: pInitialization.D
                                        * Referenced by: '<S14>/D'
                                        */
  real_T X0_Value[2];                  /* Expression: pInitialization.X0
                                        * Referenced by: '<S14>/X0'
                                        */
  real_T KalmanGainM_Value_n[4];       /* Expression: pInitialization.M
                                        * Referenced by: '<S77>/KalmanGainM'
                                        */
  real_T gravity_Value_b[3];           /* Expression: [0 0 -quad.g]
                                        * Referenced by: '<S70>/gravity'
                                        */
  real_T gainaccinput_Gain;            /* Expression: estimParams.pos.accelerationInput_gain
                                        * Referenced by: '<S70>/gainaccinput'
                                        */
  real_T C_Value_j[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S71>/C'
                                        */
  real_T D_Value_b[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S71>/D'
                                        */
  real_T X0_Value_j[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S71>/X0'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S3>/Delay1'
                                        */
  real_T C_Value_f[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S134>/C'
                                        */
  real_T D_Value_o[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S134>/D'
                                        */
  real_T X0_Value_k[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S134>/X0'
                                        */
  real_T SimplyIntegrateVelocity_gainval;/* Computed Parameter: SimplyIntegrateVelocity_gainval
                                          * Referenced by: '<S69>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_IC;   /* Expression: 0
                                        * Referenced by: '<S69>/SimplyIntegrateVelocity'
                                        */
  real_T SimplyIntegrateVelocity_UpperSa;/* Expression: estimParams.pos.veloIntegrator_max
                                          * Referenced by: '<S69>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_LowerSa;/* Expression: -estimParams.pos.veloIntegrator_max
                                          * Referenced by: '<S69>/SimplyIntegrateVelocity'
                                          */
  real_T UseIPPosSwitch_Threshold;     /* Expression: 0
                                        * Referenced by: '<S69>/UseIPPosSwitch'
                                        */
  real_T Bias_Bias;                    /* Expression: estimParams.alt.deltaSonarToCurrent_max
                                        * Referenced by: '<S10>/Bias'
                                        */
  real_T Bias1_Bias;                   /* Expression: -estimParams.alt.deltaSonarToCurrent_max
                                        * Referenced by: '<S10>/Bias1'
                                        */
  real_T A_Value[4];                   /* Expression: pInitialization.A
                                        * Referenced by: '<S14>/A'
                                        */
  real_T B_Value[2];                   /* Expression: pInitialization.B
                                        * Referenced by: '<S14>/B'
                                        */
  real_T KalmanGainL_Value[2];         /* Expression: pInitialization.L
                                        * Referenced by: '<S19>/KalmanGainL'
                                        */
  real_T A_Value_m[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S71>/A'
                                        */
  real_T B_Value_b[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S71>/B'
                                        */
  real_T KalmanGainL_Value_p[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S77>/KalmanGainL'
                                        */
  real_T A_Value_g[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S134>/A'
                                        */
  real_T B_Value_a[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S134>/B'
                                        */
  real_T KalmanGainL_Value_o[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S137>/KalmanGainL'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S69>/Delay'
                                        */
  uint32_T Delay_DelayLength_i;        /* Computed Parameter: Delay_DelayLength_i
                                        * Referenced by: '<S68>/Delay'
                                        */
  uint32_T Delay2_DelayLength;         /* Computed Parameter: Delay2_DelayLength
                                        * Referenced by: '<S10>/Delay2'
                                        */
  uint32_T MemoryX_DelayLength;        /* Computed Parameter: MemoryX_DelayLength
                                        * Referenced by: '<S14>/MemoryX'
                                        */
  uint32_T MemoryX_DelayLength_e;      /* Computed Parameter: MemoryX_DelayLength_e
                                        * Referenced by: '<S71>/MemoryX'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S3>/Delay1'
                                        */
  uint32_T MemoryX_DelayLength_a;      /* Computed Parameter: MemoryX_DelayLength_a
                                        * Referenced by: '<S134>/MemoryX'
                                        */
  P_ControllerPolePlace_Drone_C_T ControllerPolePlace;/* '<S1>/ControllerPolePlace' */
};

/* Parameters (auto storage) */
struct P_Drone_Compensator_T_ {
  struct_fgpOXIBQXBGEsNX5uiZ1j quadEDT;/* Variable: quadEDT
                                        * Referenced by:
                                        *   '<S3>/sampleTime'
                                        *   '<S10>/prsToAlt_gain'
                                        *   '<S13>/inverseIMU_gain'
                                        *   '<S9>/ThrustToW2_Gain'
                                        *   '<S9>/W2ToMotorsCmd_Gain'
                                        *   '<S15>/SaturationSonar'
                                        *   '<S187>/Constant'
                                        *   '<S64>/Constant'
                                        */
  struct_pP0yJPvqYhejK9gHgcbWI quad;   /* Variable: quad
                                        * Referenced by:
                                        *   '<S3>/sampleTime1'
                                        *   '<S8>/HoverThrustLinearizationPoint'
                                        */
  struct_SNZkZ6rc9vhdEg27lKQdHF controlHelperParams;/* Variable: controlHelperParams
                                                     * Referenced by:
                                                     *   '<S7>/TorquetotalThrustToThrustperMotor'
                                                     *   '<S7>/Saturation2'
                                                     *   '<S8>/takeoff_Gain'
                                                     *   '<S8>/SaturationThrust'
                                                     */
  real_T K_poleplace[48];              /* Variable: K_poleplace
                                        * Referenced by: '<S4>/Gain'
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
 * '<S2>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace'
 * '<S3>'   : 'sim_quadrotor/Drone_Compensator/Estimator'
 * '<S4>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace/FullstateController'
 * '<S5>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace/SysteminputConverter'
 * '<S6>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace/statesReferences'
 * '<S7>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace/SysteminputConverter/ControlMixer'
 * '<S8>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace/SysteminputConverter/Takeoffphase_Thrustadjustment'
 * '<S9>'   : 'sim_quadrotor/Drone_Compensator/ControllerPolePlace/SysteminputConverter/Thrust2Motorcmd'
 * '<S10>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude'
 * '<S11>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorOrientation'
 * '<S12>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition'
 * '<S13>'  : 'sim_quadrotor/Drone_Compensator/Estimator/SensorPreprocessing'
 * '<S14>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude'
 * '<S15>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling'
 * '<S16>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/outlierBelowFloor'
 * '<S17>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/trafo_BodytoWorld_trans'
 * '<S18>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/trafo_WorldToBody_trans'
 * '<S19>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL'
 * '<S20>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculateYhat'
 * '<S21>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionA'
 * '<S22>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionB'
 * '<S23>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionC'
 * '<S24>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionD'
 * '<S25>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionG'
 * '<S26>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionH'
 * '<S27>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionN'
 * '<S28>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP'
 * '<S29>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP0'
 * '<S30>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionQ'
 * '<S31>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionR'
 * '<S32>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionReset'
 * '<S33>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX'
 * '<S34>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX0'
 * '<S35>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionu'
 * '<S36>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/MemoryP'
 * '<S37>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer'
 * '<S38>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ReducedQRN'
 * '<S39>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionP0'
 * '<S40>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionQ'
 * '<S41>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionR'
 * '<S42>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator'
 * '<S43>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkA'
 * '<S44>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkB'
 * '<S45>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkC'
 * '<S46>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkD'
 * '<S47>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkEnable'
 * '<S48>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkG'
 * '<S49>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkH'
 * '<S50>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkN'
 * '<S51>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkP0'
 * '<S52>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkQ'
 * '<S53>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkR'
 * '<S54>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkReset'
 * '<S55>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkX0'
 * '<S56>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checku'
 * '<S57>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checky'
 * '<S58>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionL'
 * '<S59>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionM'
 * '<S60>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionP'
 * '<S61>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionZ'
 * '<S62>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer/MeasurementUpdate'
 * '<S63>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator/Enabled Subsystem'
 * '<S64>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/check for min altitude'
 * '<S65>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentEstimateVeryOffFromPressure'
 * '<S66>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffsonarflt'
 * '<S67>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierJump'
 * '<S68>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity'
 * '<S69>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition'
 * '<S70>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling'
 * '<S71>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy'
 * '<S72>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling'
 * '<S73>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/Deactivate Acceleration If OF is not used due to low altitude'
 * '<S74>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/do not use acc if optical flow never available (Note OF@60Hz but ZOH to 200!)'
 * '<S75>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/do not use acc if optical flow never available (Note OF@60Hz but ZOH to 200!)1'
 * '<S76>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationHandling/trafo_World2Body_trans'
 * '<S77>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL'
 * '<S78>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculateYhat'
 * '<S79>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionA'
 * '<S80>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionB'
 * '<S81>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionC'
 * '<S82>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionD'
 * '<S83>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionG'
 * '<S84>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionH'
 * '<S85>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionN'
 * '<S86>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP'
 * '<S87>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP0'
 * '<S88>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionQ'
 * '<S89>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionR'
 * '<S90>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionReset'
 * '<S91>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX'
 * '<S92>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX0'
 * '<S93>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionu'
 * '<S94>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/MemoryP'
 * '<S95>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer'
 * '<S96>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ReducedQRN'
 * '<S97>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionP0'
 * '<S98>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionQ'
 * '<S99>'  : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionR'
 * '<S100>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator'
 * '<S101>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkA'
 * '<S102>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkB'
 * '<S103>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkC'
 * '<S104>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkD'
 * '<S105>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkEnable'
 * '<S106>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkG'
 * '<S107>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkH'
 * '<S108>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkN'
 * '<S109>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkP0'
 * '<S110>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkQ'
 * '<S111>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkR'
 * '<S112>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkReset'
 * '<S113>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkX0'
 * '<S114>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checku'
 * '<S115>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checky'
 * '<S116>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionL'
 * '<S117>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionM'
 * '<S118>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionP'
 * '<S119>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionZ'
 * '<S120>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer/MeasurementUpdate'
 * '<S121>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator/Enabled Subsystem'
 * '<S122>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/DiscreteDerivative'
 * '<S123>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxdw1'
 * '<S124>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxdw2'
 * '<S125>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxp'
 * '<S126>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxp2'
 * '<S127>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxq'
 * '<S128>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxq2'
 * '<S129>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw1'
 * '<S130>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw2'
 * '<S131>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw3'
 * '<S132>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/maxw4'
 * '<S133>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/UglyDataHandling/minHeightforOF'
 * '<S134>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy'
 * '<S135>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling'
 * '<S136>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/trafo_BodytoWorld_trans'
 * '<S137>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL'
 * '<S138>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculateYhat'
 * '<S139>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionA'
 * '<S140>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionB'
 * '<S141>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionC'
 * '<S142>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionD'
 * '<S143>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionG'
 * '<S144>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionH'
 * '<S145>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionN'
 * '<S146>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP'
 * '<S147>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP0'
 * '<S148>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionQ'
 * '<S149>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionR'
 * '<S150>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionReset'
 * '<S151>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX'
 * '<S152>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX0'
 * '<S153>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionu'
 * '<S154>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/MemoryP'
 * '<S155>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer'
 * '<S156>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ReducedQRN'
 * '<S157>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionP0'
 * '<S158>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionQ'
 * '<S159>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionR'
 * '<S160>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator'
 * '<S161>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkA'
 * '<S162>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkB'
 * '<S163>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkC'
 * '<S164>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkD'
 * '<S165>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkEnable'
 * '<S166>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkG'
 * '<S167>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkH'
 * '<S168>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkN'
 * '<S169>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkP0'
 * '<S170>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkQ'
 * '<S171>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkR'
 * '<S172>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkReset'
 * '<S173>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkX0'
 * '<S174>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checku'
 * '<S175>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checky'
 * '<S176>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionL'
 * '<S177>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionM'
 * '<S178>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionP'
 * '<S179>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionZ'
 * '<S180>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer/MeasurementUpdate'
 * '<S181>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator/Enabled Subsystem'
 * '<S182>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs'
 * '<S183>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/checkifPosavailable'
 * '<S184>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxp3'
 * '<S185>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxq3'
 * '<S186>' : 'sim_quadrotor/Drone_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/planarjumpsVISPOS'
 * '<S187>' : 'sim_quadrotor/Drone_Compensator/Estimator/SensorPreprocessing/Check if valid visual position estimate available'
 */
#endif                                 /* RTW_HEADER_Drone_Compensator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
