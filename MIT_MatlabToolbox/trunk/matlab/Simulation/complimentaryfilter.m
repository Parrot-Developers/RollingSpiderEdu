%% Estimating attitude
function [yaw,pitch,roll,w_euler,forceMagnitudeApprox]=complimentaryfilter(yaw_cur, pitch_cur, roll_cur, g,dt,sensorIMU_datin, yawVIS_datin, validVIS_flagin,accvsGyro_f)
% ===============================
% PURPOSE estimates the attitude fusing gyroscopic and accelerometer measurements with potentially available vision updates on yaw
% INSPIRED by http://www.pieter-jan.com/node/11 
% ADAPTED by Fabian Riether
% UPDATE DATE 2015/08/25
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================

%sensorupdates 1-3 acc[1-3], 4-6 gyr[1-3]
  
GYROSCOPE_SENSITIVITY = 1;      %65.536;

%Rotation of angular velocity vector from Bodyframe to Worldframe, inverted Wronskian (body rates p-q-r to euler rates yaw pitch roll)
iW = [0        sin(roll_cur)          cos(roll_cur);             
      0        cos(roll_cur)*cos(pitch_cur) -sin(roll_cur)*cos(pitch_cur);
      cos(pitch_cur) sin(roll_cur)*sin(pitch_cur) cos(roll_cur)*sin(pitch_cur)] / cos(pitch_cur);

w_euler = iW*sensorIMU_datin(4:6);

%Integrate gyroscope data
roll  = roll_cur  + (w_euler(3) / GYROSCOPE_SENSITIVITY) * dt; %Angle around pitch_cur X-axis
pitch = pitch_cur + (w_euler(2) / GYROSCOPE_SENSITIVITY) * dt; %Angle around pitch_cur Y-axis
yaw  = yaw_cur  + (w_euler(1) / GYROSCOPE_SENSITIVITY) * dt;   %Angle around pitch_cur Z-axis  
    
%Compensate for drift with accelerometer data if un-accelerated flight
forceMagnitudeApprox = norm(sensorIMU_datin(1:3));
acceptanceGyroControl = 0.007; %0.03

if (forceMagnitudeApprox > (1-acceptanceGyroControl)*(g) && forceMagnitudeApprox < (1+acceptanceGyroControl)*(g))    
    rollAcc = atan2(sensorIMU_datin(3), -sensorIMU_datin(2))+pi/2;
    roll = roll * (1-accvsGyro_f) + rollAcc * accvsGyro_f;
    %pitchAcc = atan2(sensor_updates(1), sensor_updates(3)) ;
    pitchAcc = cos(roll)*(atan2(sensorIMU_datin(3), sensorIMU_datin(1))+pi/2) ;
    pitch = pitch * (1-accvsGyro_f) + pitchAcc *accvsGyro_f;
end;

%compensate yaw-bias/drift to world yaw if measurement form vision available
if (validVIS_flagin)
    yaw = 0.98*yaw + 0.02*(yawVIS_datin);
end;
        
end