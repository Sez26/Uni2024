

% get e and t from timeseries data
function [e_th1, e_th2] = get_Data_from(workspace_name, L1, L2)
    error1 = workspace_name.get('e_th1');
    error2 = workspace_name.get('e_th2');
    error1_vals = error1.Values
    error2_vals = error2.Values
    e_th1 = error1_vals.Data;
    e_th2 = error2_vals.Data;
end

[e_th1, e_th2] = get_Data_from(Plant_0, L1, L2);

%IAE = trapz(t, abs(e)); % t is the time vector, e is the error vector

% Plot IAE against time
%plot(IAE_0, t,'k', 'LineWidth', 0.5)