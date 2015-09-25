%% Linearization of drone dynamics about hover & full-state feedback design
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This code assists in linearizing drone dynamics and designing
% full-state feedback controls
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================

%Load drone parameters from RoboticsToolbox
mdl_quadrotor


%% 1.1) Simplified Dynamics

%symbolic variables
syms Pxw Pyw Pzw yaw pitch roll dpx dpy dpz p q r T tauy taup taur;
symsvector  = [Pxw; Pyw; Pzw ;yaw ;pitch ;roll ;dpx ;dpy ;dpz ;p ;q ;r ;T ;tauy ;taup ;taur];

%Transform inertia from RTB frame to RS frame
J           = ([cos(-pi/4) -sin(-pi/4) 0; sin(-pi/4) cos(-pi/4) 0; 0 0 1]'*quad.J*[cos(-pi/4) -sin(-pi/4) 0; sin(-pi/4) cos(-pi/4) 0; 0 0 1]);

%Define Rotation matrices
Ryaw = [
	[ cos(yaw), -sin(yaw), 0],
	[ sin(yaw),  cos(yaw), 0],
	[        0,         0, 1]
];

Rpitch = [
	[  cos(pitch), 0, sin(pitch)],
	[           0, 1,          0],
	[ -sin(pitch), 0, cos(pitch)]
];

Rroll = [
	[ 1,         0,          0],
	[ 0, cos(roll), -sin(roll)],
	[ 0, sin(roll),  cos(roll)]
];

Body2Global = Ryaw*Rpitch*Rroll;
Global2Body = simplify(Body2Global^-1);

%inverted Wronskian to transform body rates p-q-r to euler rates yaw pitch roll
iW = ...
    [0        sin(roll)          cos(roll);             
     0        cos(roll)*cos(pitch) -sin(roll)*cos(pitch);
     cos(pitch) sin(roll)*sin(pitch) cos(roll)*sin(pitch)] / cos(pitch);

%%Linearization Point = Hover
%-----------
state_equil = [0; 0; -1.5; 0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ];
input_equil = [-quad.g*quad.M ;0 ;0 ;0];
equil       = [state_equil; input_equil];

%%Dynamics
%----------      
%P dot      
P_dot           = Body2Global*[dpx;dpy;dpz];
P_dot_jacobian  = jacobian(P_dot,symsvector);
P_dot_jacobian_eql = subs(P_dot_jacobian,symsvector,equil);

%O dot      
O_dot           = iW*[p;q;r];
O_dot_jacobian  = jacobian(O_dot,symsvector);
O_dot_jacobian_eql = subs(O_dot_jacobian,symsvector,equil);

%p ddot      
p_ddot          = Global2Body*[0;0;quad.g] + T/quad.M*[0;0;1];
p_ddot_jacobian = jacobian(p_ddot,symsvector);
p_ddot_jacobian_eql = subs(p_ddot_jacobian,symsvector,equil);
%o ddot      
o_ddot          = inv(J)*([taur; taup; tauy] - cross([p;q;r],J*[p;q;r]));
o_ddot_jacobian = jacobian(o_ddot,symsvector);
o_ddot_jacobian_eql = subs(o_ddot_jacobian,symsvector,equil);

%Dynamics matrix
%---------- 

matrixAB = [P_dot_jacobian_eql;O_dot_jacobian_eql;p_ddot_jacobian_eql;o_ddot_jacobian_eql];
A = double(matrixAB(1:12,1:12))
B = double(matrixAB(1:12,13:16))
x_0 = state_equil
u_0 = input_equil

%% 1.2) Linearizing Full Nonlinear Simulink Model (the model from Robotics Toolbox)
%use Simulation/controllers/controller_fullstate/linearizeDrone.slx and Simulink's ControlDesign/Linear Analysis

%% 2.0) Load Full-state Feedback Controller derived from the PIDtoW-controller
%(see linearizePID2W.m)
K_pid = [0 0 0.425862895347363 0 0 0 0 0 0.248420022285962 0 0 0;0 0 0 8.28435779424437e-05 0 0 0 0 0 0 0 2.48530733827331e-05;-0.00398603848919041 0 0 0 0.013286794963968 0 -0.0019930192445952 0 0 0 0.0017715726618624 0;0 0.00863641672657921 0 0 0 0.028788055755264 0 0.00431820836328961 0 0.00664339748198401 0 0];

% Generate c-code ready format for copy-paste straight into src-files rsedu_control.c
K_pid_ccode_string = sprintf('%E,' , K_pid(:));
K_pid_ccode_string = ['{ ' K_pid_ccode_string(1:end-1) ' }']

%% 2.1) Designing Full-state Feedback Controllers with Simplified Dynamics Model (1.1) via Pole Placement

% Find states to decouple

%CODE MISSING

% Check what system looks like with K_pid-fullstate-feedback from (2.0)

%CODE MISSING

% Now place your own poles

%CODE MISSING

% Compute Full-state feedback for original system

%CODE MISSING

% Generate c-code ready format for copy-paste straight into src-files rsedu_control.c
K_poleplace_string = sprintf('%E,' , K_poleplace(:));
K_poleplace_string = ['{ ' K_poleplace_string(1:end-1) ' }']

% Check where poles land
% [V,E,]=eig(A_dec_x-B_dec_x*K_dec_x);
% [V,E,]=eig(A_dec_y-B_dec_y*K_dec_y);


%% 2.2) Designing Full-state Feedback Controllers with Simplified Dynamics Model (1.1) via LQR

%CODE MISSING

K_LQR       = lqr(A,B,Q,R);
K_LQR(abs(K_LQR)<(1e-10))=0;  %set small values zero
K_LQR(2,:) = K_poleplace(2,:); %LQR does not work well on yaw as LQR places much weight on yaw-rate (which is quite noisy)

% Generate c-code ready format for copy-paste to src-files.
K_LQR_ccode_string = sprintf('%E,' , K_LQR(:));
K_LQR_ccode_string = ['{ ' K_LQR_ccode_string(1:end-1) ' }']



