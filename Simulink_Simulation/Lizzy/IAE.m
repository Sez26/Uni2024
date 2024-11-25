% get e and t from timeseries data
function [e_th1, e_th2, time] = get_Data_from(workspace_name)
    error1 = workspace_name.get('e_th1');
    error2 = workspace_name.get('e_th2');
    error1_vals = error1.Values;
    error2_vals = error2.Values;
    e_th1 = error1_vals.Data;
    e_th2 = error2_vals.Data;
    time = error1_vals.Time; %verified both time series are the same
end

function plot_this(plant)
    [e_th1, e_th2, time] = get_Data_from(plant);      %get errors from timeseries data
    e_th1 = abs(deg2rad(e_th1)); e_th2 = abs(deg2rad(e_th2));             %turn errors into absolute values
    
    if time(2)-time(1) == time(3)-time(2) %time steps are the same between each point
        dt = time(2) - time(1); % Time difference between consecutive points
        IAE_th1 = cumsum(e_th1) * dt;
        IAE_th2 = cumsum(e_th2) * dt;
    else %non-linear time: time steps vary between each point
        dt = diff(time);
        IAE_contribution_th1 = [0; e_th1(2:end) .* dt];
        IAE_contribution_th2 = [0; e_th2(2:end) .* dt];
        IAE_th1 = cumsum(IAE_contribution_th1);
        IAE_th2 = cumsum(IAE_contribution_th2);
    end
    plot(time, IAE_th1, 'LineWidth', 1)
    plot(time, IAE_th2, 'LineWidth', 1)

end

% Plot IAE against time
figure; hold on;
plot_this(Plant_0_circle);
plot_this(Plant_0_square);
plot_this(Plant_0_triangle);
xlabel('Time (s)');
ylabel('IAE (rad.s)');
legend('Circle θ_1', 'Circle θ_2','Square θ_1', 'Square θ_2','Triangle θ_1','Triangle θ_2');
title('Non linear model: Integral of Absolute Error')