%% Estimating attitude
function [yaw,pitch,roll,w_euler,imu_abs]=complimentaryfilter(yaw_old, pitch_old, roll_old, g, dt,sensorIMU_datin, yawVIS_datin, validVIS_flagin)
% ===============================
% PURPOSE estimates the orientation fusing gyroscopic and accelerometer measurements with potentially available vision updates on yaw
% INSPIRED by http://www.pieter-jan.com/node/11 
% ADAPTED by Fabian Riether
% UPDATE DATE 2015/08/25
% SPECIAL NOTES
% ===============================
%  2015/08/25 created
% ==================================

%PARAMS
gyroAngleUpdate_acc_threshold   = 0.002;
gyroAngleUpdate_acc_weight      = 0.001;

gyroAngleUpdate_vis_weight      = 0.2;
%--
GYROSCOPE_SENSITIVITY = 1;

%Rotation of angular velocity vector from Bodyframe to Worldframe, inverted Wronskian (body rates p-q-r to euler rates yaw pitch roll)
iW = [0        sin(roll_old)          cos(roll_old);             
      0        cos(roll_old)*cos(pitch_old) -sin(roll_old)*cos(pitch_old);
      cos(pitch_old) sin(roll_old)*sin(pitch_old) cos(roll_old)*sin(pitch_old)] / cos(pitch_old);

w_euler = iW*sensorIMU_datin(4:6);

%Integrate gyroscope data
roll  = roll_old  + (w_euler(3) / GYROSCOPE_SENSITIVITY) * dt;
pitch = pitch_old + (w_euler(2) / GYROSCOPE_SENSITIVITY) * dt;
yaw   = yaw_old   + (w_euler(1) / GYROSCOPE_SENSITIVITY) * dt; 

  
%Compensate for drift with accelerometer data if in un-accelerated flight
imu_abs = norm(sensorIMU_datin(1:3));

if ( (imu_abs > (1-gyroAngleUpdate_acc_threshold)*(g)) && (imu_abs < (1+gyroAngleUpdate_acc_threshold)*(g)))
    roll_hat_acc = atan(sensorIMU_datin(2)/sensorIMU_datin(3));    
    roll         = roll * (1-gyroAngleUpdate_acc_weight) + roll_hat_acc * gyroAngleUpdate_acc_weight;
    
    pitch_hat_acc= asin(sensorIMU_datin(1)/g) ;
    pitch        = pitch * (1-gyroAngleUpdate_acc_weight) + pitch_hat_acc *gyroAngleUpdate_acc_weight;
end;

%Compensate yaw-bias/drift to world yaw if measurement form visually reconstructed pose based on marker setup available

if (validVIS_flagin)
    yaw = (1-gyroAngleUpdate_vis_weight)*yaw + gyroAngleUpdate_vis_weight*(yawVIS_datin);
end;
        
end