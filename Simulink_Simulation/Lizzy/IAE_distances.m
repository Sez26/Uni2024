% get errors and times from timeseries data
function [err, time] = get_Data(workspace_name, L1, L2, nonlinear, start)
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
    time = time(start:end);

    %Calculate error distances
    response = [xData_th1(start:end), yData_th2(start:end)];
    ref = [xData_ref1(start:end), yData_ref2(start:end)];
    err = sqrt(sum((response - ref).^2, 2));
end

%plot IAE graph
function [time, IAE] = plot_this(plant, L1, L2, nonlinear,start)
    [err, time] = get_Data(plant, L1, L2, nonlinear,start); %get errors from timeseries data

    %delete first items in err and time
    err = err(start:end);
    time = time(start:end);

    if time(2)-time(1) == time(3)-time(2) %time steps are the same between each point
        dt = time(2) - time(1); % Time difference between consecutive points
        IAE = cumsum(err) * dt;
    else %non-linear time: time steps vary between each point
        dt = diff(time);
        IAE_contribution = [0; err(2:end) .* dt];
        IAE = cumsum(IAE_contribution);
    end
    semilogy(time, IAE, 'LineWidth', 1.5);
end

% Plot IAE against time
clear figure;

figure('Renderer', 'painters', 'Position', [10 10 550 500]);
hold on; grid on;

start = 1;
%[time, IAE] = plot_this(PD_model, L1, L2, false, start);
[time, IAE] = plot_this(PID_uniform, L1, L2, false, start);
%time, IAE] = plot_this(PID_model_future_work, L1, L2, false, start);

[time, IAE] = plot_this(PID_bspline, L1, L2, false, start);
%[time, IAE] = plot_this(PID_nonlinear_bspline, L1, L2, false, start);

%[time, IAE] = plot_this(PID_square2_saturated, L1, L2, false, start);
%[time, IAE] = plot_this(PID_square2_no_sat, L1, L2, false, start);

set(gca, 'YScale', 'log');
% ylim([0.0022 0.0035]); %ymin, ymax
xlim([2 4])


xlabel('Time (s)');
ylabel('IAE (deg)');
legend('PID model uniform references','PID model bspline references');
