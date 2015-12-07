%% Rearrange saved timeseries from Simulink into data format of hardware-logged-data.

if exist('rt_yout_sim_tmp','var')
    
   rt_yout_sim.time = rt_yout_sim_tmp.signal1.Time;
   
   fields = fieldnames(rt_yout_sim_tmp);
   
   signalIndex = 1;
   
    for i = 1:numel(fields)
        if isstruct(rt_yout_sim_tmp.(fields{i}))
            fields_signalstruct = fieldnames(rt_yout_sim_tmp.(fields{i}));
            for k=1:(numel(fields_signalstruct))
               rt_yout_sim.signals(signalIndex).values = rt_yout_sim_tmp.(fields{i}).(fields_signalstruct{k}).data; 
               rt_yout_sim.signals(signalIndex).dimension = size(rt_yout_sim_tmp.(fields{i}).(fields_signalstruct{k}).data,2); 
               rt_yout_sim.signals(signalIndex).label = ''; 
               rt_yout_sim.signals(signalIndex).blockName = ''; 
               signalIndex = signalIndex + 1;
            end;           
        else
           rt_yout_sim.signals(signalIndex).values = rt_yout_sim_tmp.(fields{i}).data; 
           rt_yout_sim.signals(signalIndex).dimension = size(rt_yout_sim_tmp.(fields{i}).data,2); 
           rt_yout_sim.signals(signalIndex).label = '';
           rt_yout_sim.signals(signalIndex).blockName = ''; 
           signalIndex = signalIndex + 1;
        end
    end
    
end

clear rt_yout_sim_tmp;