%% Design and Test Filters for various sensors

% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE These scripts assist in desinging quick and dirty filters for the
% drone's sensors
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================

%% filter
   filterAccelero = designfilt('lowpassfir', 'FilterOrder',5, 'CutoffFrequency', 0.1);
data_f = filter(filterAccelero,RSrun_sensordata(:,2:end-2));
figure
plot(RSrun_sensordata(:,2:end-2));hold all;
plot(data_f,'-'); 

figure
 lpFilt_gyro = designfilt('lowpassfir', 'FilterOrder',5, 'CutoffFrequency', 0.001);
 [filter_b_gyroz,filter_a_gyroz] = cheby2(5,40,0.8);
plot(RSrun_sensordata(:,5:end-2));hold all;
plot(filter(filter_b_gyroz,filter_a_gyroz,RSrun_sensordata(:,5:end-2)),'-'); 

%% Test with data imu
data_f = filter(filterAccelero,RSrun_sensordata(:,2:7));
figure;
hold off;
plot(RSrun_sensordata(:,2:7)); hold all
plot(data_f); 


%%with altitude

figure;

   [b,a] = butter(6,0.02);
%data_f = filtfilt(lpFilt,data(:,1:6));

%% Test with hieght

[filter_b_prs,filter_a_prs] = cheby2(5,20,0.01);

figure
[b,a] = cheby2(5,80,0.05);
data = permute(rt_yout.signals(9).values,[3 2 1])-0.44;
data=[data];
%freqz(b,a)
data_f = filter(b,a,data);

data_fm = 0*data;
data_fm(1,1) = data(1,1);

for i=2:size(data,1)
    data_fm (i,1) = data_fm (i-1,1)*0.992 + data(i,1)*(1-0.992);
end
Panamericana 

hold off;
plot(data+0.44); hold all
plot(data_f+0.44);
plot(data_fm+0.44);


figure
plot(rt_tout,cumsum(cumsum(rt_yout.signals(5).values+(9.81-0.28019222923597558))*0.005)*0.005);

% %% angular vel for no-optiflow
% 
% [b_pqof,a_pqof] = cheby2(2,10,0.002); %used order 8 before
% 
% data = [rt_yout.signals(19).values,rt_yout.signals(19).values];
% filterdata = filter(b_pqof,a_pqof,data);
% hold off;
% figure;
% plot(rt_yout.time, data); hold all;
% plot(rt_yout.time, filterdata);

[filter_b_prs,filter_a_prs] = cheby2(5,20,0.01);

%% test with optical flow
figure
data = 1/quadEDT.VelocityToOpticalFlow_Gain*RSrun_opticalFlowRS(:,2:3);
hold off;
plot(rt_yout.time,data); hold all;
plot(RSrun_states_estim(:,1),RSrun_states_estim(:,8),'.-');
plot(RSrun_states_estim(:,1),RSrun_states_estim(:,9),'.-');


%[b_of,a_of] = cheby2(6,10,0.02); %before including rot vel!
[b_of,a_of] = cheby2(6,10,0.25)

fir_of = designfilt('lowpassfir', 'FilterOrder',9, 'CutoffFrequency', 0.01); %used order 8 before

data_f = filter(b_of,a_of,data);
data_2f = filter(fir_of,data);


plot(rt_yout.time,data_f,'--'); 
plot(rt_yout.time,data_2f,'.-');hold off;
xlim([7 11]);
ylim([-40e-3 60e-3]);
legend 'ofx' 'ofy' 'xhat' 'yhat' 'xiir' 'yiir' 'xfir' 'yfir'