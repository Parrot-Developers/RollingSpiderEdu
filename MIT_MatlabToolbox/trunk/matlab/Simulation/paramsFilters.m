%% Parameter File for filter setup
%... with the EducationalDroneToolbox
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This files containts settings for filters and some helpers for controls
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================

%Load drone parameters from RoboticsToolbox
mdl_quadrotor


%% Filters & Estimators
%Filters Raw Sensors
controlParams.filter_accelero                     = designfilt('lowpassfir', 'FilterOrder',5, 'CutoffFrequency', 0.1);
[altEstim.filter_b_gyroz,altEstim.filter_a_gyroz] = cheby2(5,40,0.8);
[altEstim.filter_b_prs,altEstim.filter_a_prs]     = cheby2(5,20,0.01);

%Estimator Attitude 
%accvsGyro     = 0.001;         %see Estimator/EstimatorAttitude
%acceptanceGyroControl = 0.007; %see Estimator/EstimatorAttitude/complimentaryFilter

%Estimator Altitude
%outlier-JumpInSonar
altEstim.outlierJump_UpperLimit      = 0.3;
altEstim.stateDeviationPrs_Threshold = 0.8;
altEstim.stateDeviationSonflt_Threshold = 0.4;
%Kalmanmatrices: see Estimator/EstimatorAltitude/KalmanFilter_altitude

%EstimatorPosition
%Kalmanmatrices in simulink Model!
%OpticalFlow Outlier Handling
ofhandle.pitchroll_UpperLimit  = 0.6;
ofhandle.pq_UpperLimit      = 7.0;
ofhandle.pq_UpperLimit_hov  = 0.5;
ofhandle.dpq_UpperLimit     = 80;
ofhandle.Z_UpperLimit       = -0.4; %note: z-axis facing downwards!
ofhandle.deltadxy_UpperLimit = 5; %m/s
%Vision Outlier Handling
vishandle.att_UpperLimit    = 0.18;
vishandle.deltaXY           = 0.5;



%% Control Mixer

%Ts2Q transforms thrust [Nm] for motors 1..4 to Q_desired=[totalThrust;Torqueyaw;pitch;roll]
controlParams.Ts2Q = [1 1 1 1;
    %quad.Cq/quad.Ct/2/1880 -quad.Cq/quad.Ct/2/1880 quad.Cq/quad.Ct/2/1880 -quad.Cq/quad.Ct/2/1880;
    quad.Cq/quad.Ct*quad.r -quad.Cq/quad.Ct*quad.r quad.Cq/quad.Ct*quad.r -quad.Cq/quad.Ct*quad.r;
    -quad.d*sqrt(2)/2 -quad.d*sqrt(2)/2  quad.d*sqrt(2)/2 quad.d*sqrt(2)/2; 
                       -quad.d*sqrt(2)/2  quad.d*sqrt(2)/2 quad.d*sqrt(2)/2 -quad.d*sqrt(2)/2                       
                       ];

%Q2Ts transform requested Q to thrust per motor
controlParams.Q2Ts                        = inv(controlParams.Ts2Q); 

%% Controllers (generic helpers)
controlParams.takeoff_Gain                = 0.05;
controlParams.totalThrust_maxRelative     = 0.92;
controlParams.motorsRS_UpperLimit        = 500;        
controlParams.motorsThrust_i_UpperLimit  = controlParams.motorsRS_UpperLimit*quad.Ct*quad.rho*quad.A*quad.r^2*quadEDT.motorsRSToW2_Gain;

controlParams.NO_VIS_X                    = -99.0;
controlParams.NO_VIS_YAW                  = -9.0;

%see LinearDroneAndFSFBControl.m for actual State-Space Controllers

