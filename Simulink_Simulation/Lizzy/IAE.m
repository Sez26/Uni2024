

% get e and t from timeseries data
function [e_th1, e_th2, time] = get_Data_from(workspace_name, L1, L2)
    error1 = workspace_name.get('e_th1');
    error2 = workspace_name.get('e_th2');
    error1_vals = error1.Values;
    error2_vals = error2.Values;
    e_th1 = error1_vals.Data;
    e_th2 = error2_vals.Data;
    time = error1_vals.Time; %verified both time series are the same
end

[e_th1, e_th2, time] = get_Data_from(Plant_0, L1, L2);

IAE_th1 = trapz(time, abs(e_th1));

% Plot IAE against time
plot(IAE_th1, time,'k', 'LineWidth', 0.5)