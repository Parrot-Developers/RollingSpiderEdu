%MDL_QUADCOPTER Dynamic parameters for a quadrotor.
%
% MDL_QUADCOPTER is a script creates the workspace variable quad which
% describes the dynamic characterstics of a quadrotor flying robot.
%
% Properties::
%
% This is a structure with the following elements:
%
% nrotors   Number of rotors (1x1)
% J         Flyer rotational inertia matrix (3x3)
% h         Height of rotors above CoG (1x1)
% d         Length of flyer arms (1x1)
% nb        Number of blades per rotor (1x1)
% r         Rotor radius (1x1)
% c         Blade chord (1x1)
% e         Flapping hinge offset (1x1)
% Mb        Rotor blade mass (1x1)
% Mc        Estimated hub clamp mass (1x1)
% ec        Blade root clamp displacement (1x1)
% Ib        Rotor blade rotational inertia (1x1)
% Ic        Estimated root clamp inertia (1x1)
% mb        Static blade moment (1x1)
% Ir        Total rotor inertia (1x1)
% Ct        Non-dim. thrust coefficient (1x1)
% Cq        Non-dim. torque coefficient (1x1)
% sigma     Rotor solidity ratio (1x1)
% thetat    Blade tip angle (1x1)
% theta0    Blade root angle (1x1)
% theta1    Blade twist angle (1x1)
% theta75   3/4 blade angle (1x1)
% thetai    Blade ideal root approximation (1x1)
% a         Lift slope gradient (1x1)
% A         Rotor disc area (1x1)
% gamma     Lock number (1x1)
%
%
% Notes::
% - SI units are used.
%
% References::
% - Design, Construction and Control of a Large Quadrotor micro air vehicle.
%   P.Pounds, PhD thesis, 
%   Australian National University, 2007.
%   http://www.eng.yale.edu/pep5/P_Pounds_Thesis_2008.pdf
% - This is a heavy lift quadrotor
%
% See also sl_quadrotor.

% MODEL: quadrotor

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

%--------------------
% Update:
% date: 2015/08/23
% editor: Fabian Riether
% comment: This updated version now contains parameters for Peter Corke's
% Robotics Toolbox that match Parrot's Rolling Spider, parameters for
% the EducationalDroneToolbox are appended
%--------------------

quad.nrotors = 4;                %   4 rotors
quad.g = 9.81;                   %   g       Gravity                             1x1
quad.rho = 1.184;                %   rho     Density of air                      1x1
quad.muv = 1.5e-5;               %   muv     Viscosity of air                    1x1

% Airframe
quad.M = 0.068;                 %   M       Mass                                1x1

% Ixx,yy,zz                     Flyer rotational inertia matrix     3x3
quad.J = diag([0.0686e-3 0.092e-3 0.1366e-3]);

quad.h = -(6.5+9.376)/1000;     %   h       Height of rotors above CoG          1x1
quad.d = 0.0624;                %   d       Length of flyer arms                1x1

%Rotor
quad.nb = 2;                    %   b       Number of blades per rotor          1x1
quad.r = 33/1000;               %   r       Rotor radius                        1x1

quad.c = 0.008;                 %   c       Blade chord                         1x1

quad.e = 0.0;                   %   e       Flapping hinge offset               1x1
quad.Mb = 0.0015/4;             %   Mb      Rotor blade mass                    1x1
quad.Mc = 0;                    %   Mc      Estimated hub clamp mass            1x1
quad.ec = 0;                    %   ec      Blade root clamp displacement       1x1
quad.Ib = quad.Mb*(quad.r-quad.ec)^2/4 ;        %   Ib      Rotor blade rotational inertia      1x1
quad.Ic = quad.Mc*(quad.ec)^2/4;                %   Ic      Estimated root clamp inertia        1x1
quad.mb = quad.g*(quad.Mc*quad.ec/2+quad.Mb*quad.r/2);    %   mb      Static blade moment                 1x1
quad.Ir = quad.nb*(quad.Ib+quad.Ic);            %   Ir      Total rotor inertia                 1x1

quad.Ct = 0.0107;                               %   Ct      Non-dim. thrust coefficient         1x1
quad.Cq = quad.Ct*sqrt(quad.Ct/2);              %   Cq      Non-dim. torque coefficient         1x1

quad.sigma = quad.c*quad.nb/(pi*quad.r);        %  sigma   Rotor solidity ratio                1x1
quad.thetat = 6.8*(pi/180);                     %  thetat  Blade tip angle  @TODO              1x1
quad.theta0 = 14.6*(pi/180);                    %  theta0  Blade root angle @TODO              1x1
quad.theta1 = quad.thetat - quad.theta0;        %  theta1  Blade twist angle                   1x1
quad.theta75 = quad.theta0 + 0.75*quad.theta1;  %  theta76 3/4 blade angle                     1x1
quad.thetai = quad.thetat*(quad.r/quad.e);      %  thetai  Blade ideal root approximation      1x1
if isinf(quad.thetai) quad.thetai = 10000; end;
quad.a = 5.5;                                   %  a       Lift slope gradient @TODO           1x1

% derived constants
quad.A = pi*quad.r^2;                           %  A       Rotor disc area                     1x1
quad.gamma = quad.rho*quad.a*quad.c*quad.r^4/(quad.Ib+quad.Ic);%   gamma   Lock number                         1x1

quad.b = quad.Ct*quad.rho*quad.A*quad.r^2;      %  T = b w^2
quad.k = quad.Cq*quad.rho*quad.A*quad.r^3;      %  Q = k w^2

quad.verbose = false;

quadEDT.w2ToThrust_gain = quad.Ct*quad.rho*quad.A*quad.r^2;

%% Motors
quadEDT.motors_max                  = 500;     %max command to control motors    
quadEDT.motorcommandToW2_gain       = 13840.8; %motor command for Rolling Spider (0-500) to motorspeed^2

quadEDT.thrustToMotorcommand        = 1/(quadEDT.w2ToThrust_gain*quadEDT.motorcommandToW2_gain);


%% Sensors
%Noise on sensor readings
quadEDT.noiseSensed_var       = diag([0.3 0.3 0.5 0.002 0.002 0.002 0.001 50]);

%Delay (all sensor data)
quadEDT.sensordelay                 = 1; %in samples of 200Hz;

%Bias (some assumed, simulated default)
quadEDT.sensordataCalib            = [0.09 -0.06 -9.4730 -0.0095 -0.0075 0.0015 101270.95];
quadEDT.IMUaccel_gain               = [+1.00596 +1.00383 +0.99454];
quadEDT.IMUgyro_gain                = [0.99861 1.00644 0.99997];

%Gains
quadEDT.airDensity                  = 1.225;
quadEDT.altToPrs_gain               = quad.g*quadEDT.airDensity ;
quadEDT.inverseIMU_gain             = [1./quadEDT.IMUaccel_gain 1./quadEDT.IMUgyro_gain];

%Saturations
quadEDT.altSenor_min                = 0.44;

%Battery DUMMY
quadEDT.dummy.batteryStatus         = [3.5 70];

%Vision
quadEDT.velocityToOpticalFlow_gain  = 1/20;

%% Simulation Parameters for EducationalDroneToolbox (apart from drone dynamics subsystem)
quadEDT.sampletime                  = 0.005;

%reference values
%----
%yaw
quadEDT.yawStep_amplitude           = 0.1;
quadEDT.yawStep_time                = 5.5;
quadEDT.yawStep_duration            = 2.5;
%pitch
quadEDT.pitchStep_amplitude         = 0.2; %0.1
quadEDT.pitchStep_time              = 3;
quadEDT.pitchStep_duration          = 1.5;
%roll
quadEDT.rollStep_amplitude          = 0.0; %0.1
quadEDT.rollStep_time               = 3; 
quadEDT.rollStep_duration           = 1;
%altitude
quadEDT.takeoff_duration            = 1;
quadEDT.altitude                    = -1.1;
%----

%Vision
quadEDT.NO_VIS_X                    = -99.0;
quadEDT.NO_VIS_YAW                  = -9.0;
quadEDT.dummy.posVIS_novisionavail  = [quadEDT.NO_VIS_X;0.0;0.0;quadEDT.NO_VIS_YAW ]; %drone sends this position to Drone_compensator when no marker to reconstruct position found
quadEDT.dummy.usePosVIS_flag        = 0;


%Motor Failure Simulation
quadEDT.motorFailure_time           = 10;%0.1;
quadEDT.motorFailure_duration       = 0.05;
quadEDT.motorFailure_m1             = 0;
quadEDT.motorFailure_m2             = 0;
quadEDT.motorFailure_m3             = -30;
quadEDT.motorFailure_m4             = 20;

quadEDT.motorUnknownGain_gain       = 1; %amplifies the TF from motors_datin (command 0-500) to acutal rotor speed (rad/s) used in the simulation; use as "model uncertainty"
