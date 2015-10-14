%% Linearizing PID2W Controller
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This code takes the PID controller that acted on the rotor speeds
% ws and linearizes it to find a PID controller states -> Q (total Thrust,
% torque yaw, pitch, roll)
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================

%% 1) Do linearization on your own
paramsFilters;

%From SIMULINK diagram
K_PID2act=[...
            600*0.32 0         0     0       -600     0      600*0.08 0       0    0  -80 0;
            0       -1300*0.29 0     0       0       -1300   0       -0.09*1300    0   -300 0  0;
            0               0 0     -6000   0       0       0       0       0    0   0  -0.3*6000;
            0               0 -600  0       0       0       0       0       -350 0   0  0]; %states to pitch roll yaw Thrustsingle_engine (here not total Thrust!)-action

%w_hover = sqrt(quad.M*quad.g/4/quad.b)

%that action to rotors m1..m4
act2omega =...
[   0.7071    0.7071   -1.0000   -1.0000;
   -0.7071    0.7071   -1.0000    1.000;
   -0.7071   -0.7071   -1.0000   -1.0000;
    0.7071   -0.7071   -1.0000    1.0000];

%states to ws
K_FSFB2omegaaction = act2omega*K_PID2act;

k=quad.Ct*quad.rho*quad.A*quad.r^2;

%m1 and m3 have positive thrust for pos rotation;
motorsdirection = diag([-1 1 -1 1]); 

%states to Thrust, linearized around w0, without offset
K_FSFB2motorsthrust_wooffset = 2*k*1880*motorsdirection*  K_FSFB2omegaaction; 

%offset
%K_FSFB2motorsthrust_wooffset_offsettoadd = -k*1880^2;

K_pid = -controlParams.Ts2Q*K_FSFB2motorsthrust_wooffset;
K_pid(abs(K_pid)<1e-10)=0

%% 2) Have linearization done by SIMULINK
%use Simulink's ControlDesign/Linear Analysis with linearizeController.slx
%result in linearizedPID2W
load('linearizedPID2W_x2motorcmdRS.mat');
motorcmd_hover = quad.g*quad.M*1/(quad.Ct*quad.rho*quad.A*quad.r^2)*1/quadEDT.motorsRSToW2_Gain/4;
K_pid = controlParams.Ts2Q *motorsdirection*(quad.g*quad.M/4)/motorcmd_hover*linearizedPID2W.d;
K_pid(abs(K_pid)<1e-10)=0;
K_pid = -K_pid