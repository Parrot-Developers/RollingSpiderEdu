%% Using a linearization of the drone dynamics about hover to design an LQR  hover controller
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/10/30
% PURPOSE This code assists in designing an LQR hover controller
% SPECIAL NOTES

%Load drone parameters from RoboticsToolbox
mdl_quadrotor


%% 1) Linearize Drone Model
%Use linearizeDrone_motorcmdTostate.slx and Simulink's ControlDesign/Linear Analysis
%to find a linear plant model.
%Note that we need to apply a state transformation on the results of the
%linearization.

     A = linsys1.c*linsys1.a*inv(linsys1.c);
     B = linsys1.c*linsys1.b;
     C = eye(12);
     D = zeros(12,4);

%% 2) Setup Bryson's rule 

%Limits on states
pos_max     = 1;
att_max     = 0.35;
dpos_max    = 1;
datt_max    = 1;

%Limit on control input
motor_max = 500;

%% Cost weights on states - sluggish dynamics
pos_x_wght        = 0.25/3;
pos_y_wght        = 0.25/3;
pos_z_wght        = 0.25/3; 
orient_ypr_wghts  = 0.175/3; %weights for each of the three angles of orientations(attitude)
dpos_wghts        = 0.175/3; %weights for each of the three velocities of position
dorient_pqr_wghts = 0.4  /3; %weights for each of the three angular rates of orientations(attitude)

rho = 0.05;

%% Cost weights on states - faster dynamics.

%...

%% Normalize and pack weights and limits
weights= [pos_x_wght pos_y_wght pos_z_wght orient_ypr_wghts orient_ypr_wghts orient_ypr_wghts dpos_wghts dpos_wghts dpos_wghts dorient_pqr_wghts dorient_pqr_wghts dorient_pqr_wghts];
weights=weights/sum(weights);
maxs   = [pos_max pos_max pos_max att_max att_max att_max dpos_max dpos_max dpos_max datt_max datt_max datt_max];

%% 3) Compute Q and R cost matrices
Q           = diag(weights./maxs);
R           = rho*diag(1./[motor_max motor_max motor_max motor_max]);

%% 4) Compute K_LQR
K_lqr_toMotorcmd       = lqr(A,B,Q,R);
K_lqr_toMotorcmd(abs(K_lqr_toMotorcmd)<(1e-10))=0;  %set small values zero



