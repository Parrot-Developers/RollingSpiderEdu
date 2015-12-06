%% Analyze processing times
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This plots processing times of the three drone threads 'control',
% 'optical flow' and 'image processing'
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================
cd(matlabpath);
cd ../..;
addpath(genpath([pwd '/DroneExchange']));
cd(matlabpath);
ptimes_control = importPtimes('../../DroneExchange/ptimes/pt_RSEDU_control.txt');
ptimes_ip = importPtimes('../../DroneExchange/ptimes/pt_RSEDU_image_processing.txt');
ptimes_of = importPtimes('../../DroneExchange/ptimes/pt_RSEDU_optical_flow.txt');

if nonzeros(abs(ptimes_ip(:,3))>1000000)>1
    display('Problems with recorded processing times files expected!');
end;

%% Plot

figure('Name','Access times of functions','Position',[100 100 700 150]);

inouttimes_control = getInOutTimes(ptimes_control);
inouttimes_ip = getInOutTimes(ptimes_ip);
inouttimes_of = getInOutTimes(ptimes_of);

 stairs(inouttimes_ip(:,1),1.00000002*inouttimes_ip(:,2),'g'); hold all;
%fill([xb;xb(end)],[yb; yb(1)],'g','LineStyle','none') 

 stairs(inouttimes_of(:,1),1.00000001*inouttimes_of(:,2),'b'); hold all;
%fill([xb;xb(end)],[yb; yb(1)],'b','LineStyle','none') 

  stairs(inouttimes_control(:,1),inouttimes_control(:,2),'r');
%fill([xb;xb(end)],[yb; yb(1)],'r','LineStyle','none')  

legend 'image processing' 'optical flow' 'control'
ylim([0.99999999 1.000000025])
xlim([0 1500])
xlabel({'t [ms]'},'Interpreter','latex');
set(gca,'ytick',[])
set(gca,'yticklabel',[])