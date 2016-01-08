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

 A = linsys1.a;
 B = linsys1.b;
 C = eye(12);
 D = zeros(12,4);
 
 %Note: We linearized about hover. This also implies: The control "policy"
 %to correct a position error was derived under a yaw-angle of zero!
 %If your drone yaw-drifts 90 deg and runs into a world-X-error, it will
 %still believe that pitch is the right answer to correct for this position error! You can compensate for this by
 %rotation the X-Y-error by the current yaw angle.

%% 2) Setup Bryson's rule 

%Limits on states
pos_max     = 0.5;
att_max     = 0.3;
dpos_max    = 1;
datt_max    = 1;

%Limit on control input
motor_max = 500;

%% Cost weights on states - sluggish control

pos_x_wght        = 0.1/3;
pos_y_wght        = 0.1/3;
pos_z_wght        = 0.1/3;

orient_ypr_wghts  = 0.25/3;  %weights for each of the three angles of orientations(attitude)

dpos_wghts        = 0.05/3; %weights for each of the three velocities of position

dorient_pqr_wghts = 0.6/3; %weights for each of the three angular rates of orientations(attitude)

rho = 15;

%% Cost weights on states - faster control

% ... 

%% Normalize and pack weights and limits on state costs
weights = [pos_x_wght pos_y_wght pos_z_wght orient_ypr_wghts orient_ypr_wghts orient_ypr_wghts dpos_wghts dpos_wghts dpos_wghts dorient_pqr_wghts dorient_pqr_wghts dorient_pqr_wghts];
weights =  weights/sqrt(sum(weights.^2));         %Make sure squared weights sum to 1
maxs    = [pos_max pos_max pos_max att_max att_max att_max dpos_max dpos_max dpos_max datt_max datt_max datt_max];

%% 3) Compute Q and R cost matrices
Q           = diag((weights.^2)./(maxs.^2));
R           = rho*diag(1./([motor_max motor_max motor_max motor_max].^2));

%% 4) Compute K_LQR
K_lqr_toMotorcmd       = lqr(A,B,Q,R);
K_lqr_toMotorcmd(abs(K_lqr_toMotorcmd)<(1e-8)) = 0  %set small values zero



