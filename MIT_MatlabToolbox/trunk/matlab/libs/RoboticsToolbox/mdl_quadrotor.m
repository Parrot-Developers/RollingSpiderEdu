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
quad.M = 0.068; %4or             %   M       Mass                                1x1

% Ixx,yy,zz in RTBframe       Flyer rotational inertia matrix     3x3
%quad.J = [cos(-pi/4) -sin(-pi/4) 0; sin(-pi/4) cos(-pi/4) 0; 0 0 1]*diag([0.0686e-3 0.092e-3 0.1366e-3])*[cos(-pi/4) -sin(-pi/4) 0; sin(-pi/4) cos(-pi/4) 0; 0 0 1]';
quad.J = diag([0.0686e-3 0.092e-3 0.1366e-3]);

quad.h = -(6.5+9.376)/1000;     %   h       Height of rotors above CoG          1x1
quad.d = 0.0624;  %TODO is this really the vertical d CoGtoRotor?                %   d       Length of flyer arms                1x1

%Rotor
quad.nb = 2;                      %   b       Number of blades per rotor          1x1
quad.r = 33/1000; % 0.165or;%                %   r       Rotor radius                        1x1

quad.c = 0.008;%0.018or;%                  %   c       Blade chord                         1x1

quad.e = 0.0;                    %   e       Flapping hinge offset               1x1
quad.Mb = 0.0015/4;%0.005or;                %   Mb      Rotor blade mass                    1x1
quad.Mc = 0;%0.010or; %0;%                %   Mc      Estimated hub clamp mass            1x1
quad.ec = 0;%0.004or; %0;%               %   ec      Blade root clamp displacement       1x1
quad.Ib = quad.Mb*(quad.r-quad.ec)^2/4 ;        %   Ib      Rotor blade rotational inertia      1x1
quad.Ic = quad.Mc*(quad.ec)^2/4;           %   Ic      Estimated root clamp inertia        1x1
quad.mb = quad.g*(quad.Mc*quad.ec/2+quad.Mb*quad.r/2);    %   mb      Static blade moment                 1x1
quad.Ir = quad.nb*(quad.Ib+quad.Ic);             %   Ir      Total rotor inertia                 1x1

quad.Ct = 0.0107; %  0.0048orig       %TODO true?       %   Ct      Non-dim. thrust coefficient         1x1
quad.Cq = quad.Ct*sqrt(quad.Ct/2);         %   Cq      Non-dim. torque coefficient         1x1

quad.sigma = quad.c*quad.nb/(pi*quad.r);         %   sigma   Rotor solidity ratio                1x1
quad.thetat = 6.8*(pi/180);    %@TODO  %   thetat  Blade tip angle                     1x1
quad.theta0 = 14.6*(pi/180);    %@TODO %   theta0  Blade root angle                    1x1
quad.theta1 = quad.thetat - quad.theta0;   %   theta1  Blade twist angle                   1x1
quad.theta75 = quad.theta0 + 0.75*quad.theta1;%   theta76 3/4 blade angle                     1x1
quad.thetai = quad.thetat*(quad.r/quad.e);      %   thetai  Blade ideal root approximation      1x1
if isinf(quad.thetai) quad.thetai = 10000; end;
quad.a = 5.5;        %@TODO            %   a       Lift slope gradient                 1x1

% derived constants
quad.A = pi*quad.r^2;                 %   A       Rotor disc area                     1x1
quad.gamma = quad.rho*quad.a*quad.c*quad.r^4/(quad.Ib+quad.Ic);%   gamma   Lock number                         1x1

quad.b = quad.Ct*quad.rho*quad.A*quad.r^2; % T = b w^2
quad.k = quad.Cq*quad.rho*quad.A*quad.r^3; % Q = k w^2

quad.verbose = false;

%% Motors
quadEDT.motorsRSToW2_Gain           = 13840.8; %motor command to motorspeed^2

%% Sensors
%Noise
quadEDT.noiseStatesSensed_std       = [1 1 1 1 1 1 0.0165195073635001 0.0152648883285633 0.0215786550496705 0.000652733165165932 0.000721701528439517 0.000690781425279554];
quadEDT.noiseStatesSensed_weights   = [0 0 0 0 0 0 0.05 0.05 0.05 1 1 1];

%Bias (defaults)
quadEDT.sensordataRSbias            = [0.09 -0.06 +0.337 -0.0095 -0.0075 0.0015 101270.95];
quadEDT.accelo_Gain                 = [+1.00596 +1.00383 +0.99454];
quadEDT.gyropq_Gain                 = [0.99861 1.00644];
quadEDT.gyror_Gain                  = 0.99997;

%Gains
quadEDT.airDensity                  = 1.225;
quadEDT.altToPrs_Gain               = quad.g*quadEDT.airDensity ;
quadEDT.VelocityToOpticalFlow_Gain  = 1/20;
quadEDT.inversesIMU_Gain            = [1./quadEDT.accelo_Gain 1./quadEDT.gyropq_Gain  1/quadEDT.gyror_Gain   ];

%Saturations
quadEDT.altSenor_LowerLimit            = 0.44;

%% Simulation Parameters for EducationalDroneToolbox (apart from drone dynamics subsystem)
quadEDT.sampleTime_qcsim            = 0.005;
sampleTime_qcsim                    = quadEDT.sampleTime_qcsim;

%Simulation flags
%Vision
quadEDT.SIMDUMMYposVIS_noVisionFlag             = [-99.0;0.0;0.0;-9];
quadEDT.SIMDUMMYusePosVIS_flag                  = 0;

%Motor Failure Simulation
quadEDT.motorFailure_time           = 0.1;
quadEDT.motorFailure_duration       = 0.05;
quadEDT.motorFailure_gainm2         = -30;
quadEDT.motorFailure_gainm3         = 20;
