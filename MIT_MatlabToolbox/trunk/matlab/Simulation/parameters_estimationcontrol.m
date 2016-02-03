%% Parameter File for estimation and controller setup
%... with the EducationalDroneToolbox
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This files containts settings for filters and some helpers for controls
% SPECIAL NOTES
% ===============================
%  2015/08/25 created
% ==================================

%Load drone and simulation parameters
mdl_quadrotor


%% Filters & Estimators


%IMU
estimParams.IMU.filter_accel                                    = designfilt('lowpassfir', 'FilterOrder',5, 'CutoffFrequency', 0.1);
[estimParams.IMU.filter_gyro_r_b,estimParams.IMU.filter_gyro_r_a] = cheby2(5,40,0.8);


%Orientation
%--
%see Estimator/EstimatorOrientation/complimentaryFilter 
%gyroAngleUpdate_acc_threshold   = 0.001;  
%gyroAngleUpdate_acc_weight      = 0.001;
%gyroAngleUpdate_vis_weight      = 0.02;


%Altitude
%--
[estimParams.alt.filter_prs_b,estimParams.alt.filter_prs_a]         = cheby2(5,20,0.01);
[estimParams.alt.filter_sonar_b,estimParams.alt.filter_sonar_a]     = cheby2(5,20,0.01);

%KF
estimParams.alt.kf.G            = [0;1];
estimParams.alt.kf.H            = 0;
estimParams.alt.kf.Q            = 0.0005;
estimParams.alt.kf.R            = [0.1];
estimParams.alt.kf.N            = 0;

%outlier Thresholds
estimParams.alt.deltaSonarToCurrent_max             = 0.3;
estimParams.alt.deltaPrsToCurrent_threshold         = 0.8;
estimParams.alt.deltaSonarToFiltered_threshold      = 0.4;

%Position
%--
estimParams.pos.opticalFlow_Z_max           = -0.4; %DO not use optical flow below!
estimParams.pos.accelerationInput_gain      = 0.2; %@TODO doesnt fully feed in acceleration!
estimParams.pos.opticalFlowToVelocity_gain  = 20;


%OpticalFlow Outlier Handling
estimParams.pos.of_pitchroll_max   = 0.6;
estimParams.pos.of_pq_max          = 7.0;
estimParams.pos.of_pq_hov_max      = 0.5;
estimParams.pos.of_dpq_max         = 80;

estimParams.pos.deltadxy_max = 5; %m/s

%KF velocity
estimParams.pos.kfvelo.G            = eye(2);
estimParams.pos.kfvelo.H            = 0;
estimParams.pos.kfvelo.Q            = 0.09.*eye(2);
estimParams.pos.kfvelo.R            = 5*eye(2);
estimParams.pos.kfvelo.N            = 0;

%KF position
estimParams.pos.kfpos.G            = 0.1*eye(2);
estimParams.pos.kfpos.H            = 0;
estimParams.pos.kfpos.Q            = 0.001*eye(2);
estimParams.pos.kfpos.R            = 0.3*eye(2);
estimParams.pos.kfpos.N            = 0;

%Visually reconstructed position from marker setup - Outlier Handling
estimParams.pos.vis_orient_max        = 0.18;
estimParams.pos.vis_deltaXY_max       = 1;

%% Control Mixer

%Ts2Q transforms thrust [Nm] for motors 1..4 to u_mechanical =[totalThrust;Torqueyaw;pitch;roll]
controlHelperParams.Ts2Q = ...
        [1 1 1 1;    
        quad.Cq/quad.Ct*quad.r -quad.Cq/quad.Ct*quad.r quad.Cq/quad.Ct*quad.r -quad.Cq/quad.Ct*quad.r;
        -quad.d*sqrt(2)/2 -quad.d*sqrt(2)/2  quad.d*sqrt(2)/2 quad.d*sqrt(2)/2; 
        -quad.d*sqrt(2)/2  quad.d*sqrt(2)/2 quad.d*sqrt(2)/2 -quad.d*sqrt(2)/2                       
         ];

%Q2Ts transform requested Q to thrust per motor
controlHelperParams.Q2Ts = inv(controlHelperParams.Ts2Q); 

%% Controllers (generic helpers)
controlHelperParams.takeoff_gain                = 0.12;   %drone takes off with constant thrust x% above hover thrust
controlHelperParams.totalThrust_maxRelative     = 0.92;   %relative maximum total thrust that can be used for gaining altitude; rest is buffer for orientation control
controlHelperParams.motorsThrustperMotor_max    = quadEDT.motors_max*quadEDT.motorcommandToW2_gain*quad.Ct*quad.rho*quad.A*quad.r^2;

%PID: parameters in simulink-model
%Poleplace: see separate files for parameters
%LQR: see separate files for parameters

