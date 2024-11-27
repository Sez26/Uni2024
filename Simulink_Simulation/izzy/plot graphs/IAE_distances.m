% get e and t from timeseries data
function [err, time] = get_Data_from(workspace_name, L1, L2, nonlinear)
    if nonlinear
        th1Data = workspace_name.get('th1').Values.Data(:,1);
        th2Data = workspace_name.get('th1').Values.Data(:,2);
    else
        th1Data = workspace_name.get('th1').Values.Data;
        th2Data = workspace_name.get('th2').Values.Data;
    end
    
    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2 = (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;

    %Import reference x y positions
    th1Data = workspace_name.get('ref_th1').Values.Data; 
    th2Data = workspace_name.get('ref_th2').Values.Data;
    xData_ref1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_ref2 = (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;

    %Return time
    time = workspace_name.get('ref_th1').Values.Time; %verified both time series are the same

    %Calculate error distances
    response = [xData_th1, yData_th2];
    ref = [xData_ref1, yData_ref2];
    err = sqrt(sum((response - ref).^2, 2));
end

function plot_this(plant, L1, L2, nonlinear)
    [err, time] = get_Data_from(plant, L1, L2, nonlinear); %get errors from timeseries data
    
    if time(2)-time(1) == time(3)-time(2) %time steps are the same between each point
        dt = time(2) - time(1); % Time difference between consecutive points
        IAE = cumsum(err) * dt;
    else %non-linear time: time steps vary between each point
        dt = diff(time);
        IAE_contribution = [0; err(2:end) .* dt];
        IAE = cumsum(IAE_contribution);
    end
    plot(time, IAE, 'LineWidth', 1)
end

% Plot IAE against time
clear figure;
figure; hold on; grid on;

plot_this(with_filter, L1, L2, false);
plot_this(no_filter, L1, L2, false);

xlabel('Time (s)');
ylabel('IAE (rad.s)');
legend('with filter','no filter');
title('Integral of Absolute Error')

% figure;
% plot_this(Plant_2_1, L1, L2,true); %hidden by Plant_3
% xlabel('Time (s)');
% ylabel('IAE (rad.s)'); 
% title('State feedback control for a square reference')
