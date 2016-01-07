%% Parameter Identification Parrot Rolling Spider
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This files contains some code to identify parameters of Parrot's Rolling Spider needed to run
% Peter Corke's drone simulator (Robotics Toolbox) and some features of
% the EducationalDroneToolbox
% SPECIAL NOTES
% ===============================
%  2015/08/25 created
% ==================================

%% Inertia

inertia_box         = @(m,ls) 1/12*m*([ls(1)^2+ls(2)^2;ls(2)^2+ls(3)^2;ls(1)^2+ls(3)^2]);
inertia_cyl_up      = @(m,l,r) [1/4*m*r^2 + 1/12*m*l^2; 1/4*m*r^2 + 1/12*m*l^2; 0.5*m*r^2];
inertia_tube_rolling= @(m,l,r1,r2) m*[1/12*(3*(r1^2+r2^2) + l^2); (r1^2+r2^2)/2 ;1/12*(3*(r1^2+r2^2) + l^2)];

mass_batt  = 0.011;
mass_frame = 0.033;
mass_motor = 0.006;
mass_wheel = 0.00465;
mass_strut = 0.0016;


inertia_batt  = inertia_box(mass_batt,[0.026,0.009,0.043]);
inertia_frame = inertia_box(mass_frame,[0.035,0.02,0.048]);
inertia_motor = inertia_cyl_up(mass_motor,0.02,0.008);
inertia_wheel = inertia_tube_rolling(mass_wheel,0.003,0.07,0.05);
inertia_strut = [1 0 0;0 0 1;1 0 0]*(inertia_cyl_up(mass_strut,0.07,0.002));

dx = 0.043;
dy = 0.043;

inertia_total = inertia_batt + inertia_frame + inertia_strut + 4*( (inertia_motor) +mass_motor*[dy^2; dx^2; dx^2+dy^2]) + 2*(inertia_wheel + mass_wheel*[0;0;dy^2]);