%% Find Parameters and Matrices needed for reconstructing camera pose on the fly

%% Landmarkpositions
%lndmrk_pos = [0 .225 .05 0.225; 0 .05 -.15 -.15; 0 0 0 0 ; 1 1 1 1 ]; %green-pink-red-blue, green origin; 3d landmarkpositions X;Y;Z;1 in homogenous coordinates
lndmrk_pos = [-0.24 .145 -0.24 0.135; 0.14 .13 -.23 -.22; 0 0 0 0 ; 1 1 1 1 ]; %green-pink-red-blue, green origin; 3d landmarkpositions in homogenous coordinates



%% Generate required matrices for 
%Load camera calibration
load('calibration/camcalbration.mat');

%Compute matrices (use those in embedded code!)
format long;
lndmrk_pinv = pinv(lndmrk_pos)
intrMatrx_it = inv(cameraParams.IntrinsicMatrix')
format short






%% Test with landmarks from camera calibration boards

% %world coordinates of some detected features on a camera calibration board
% Xw  = [cameraParams.WorldPoints';zeros(1,size(cameraParams.WorldPoints,1));ones(1,size(cameraParams.WorldPoints,1))];
% Xw = Xw(:,1:50);
% 
% %pixel positions of those features
% feature_pps = [cameraParams.ReprojectedPoints(1:50,:,1)';ones(1,50)];
% 
% %reconstruction assuming only yaw angle
% A_rcn_tmp = intrMatrx_it*feature_pps*pinv(Xw);
% A_rcn = A_rcn_tmp/sqrt(A_rcn_tmp(1,1)^2+A_rcn_tmp(1,2)^2);
% 
% X=A_rcn(1,4)
% Y=A_rcn(2,4)
% Z=A_rcn(3,4)
% yaw=acos(A_rcn(1,1))