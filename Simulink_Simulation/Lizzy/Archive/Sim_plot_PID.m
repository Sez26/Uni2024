% Create figure
clear figure;
figure;
hold on;

function [xData, yData] = get_Data_from(workspace_name, L1, L2)
    theta1Signal = workspace_name.get('theta1'); % Get 'theta1' signal
    theta2Signal = workspace_name.get('theta2'); % Get 'theta2' signal
    theta1Timeseries = theta1Signal.Values;
    theta2Timeseries = theta2Signal.Values;
    th1Data = theta1Timeseries.Data;
    th2Data = theta2Timeseries.Data;
    xData = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData = (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
end

%Plot P
% [xData_P, yData_P] = get_Data_from(P_control, L1, L2);
% plot(xData_P, yData_P, 'c', 'LineWidth', 0.5);
%h = plot(xData_P(1), yData_P(1), 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c'); % Initial point

%Plot PI
% [xData_PI, yData_PI] = get_Data_from(PI_control, L1, L2);
% plot(xData_PI, yData_PI, 'c', 'LineWidth', 0.5);
%h = plot(xData_P(1), yData_P(1), 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c'); % Initial point

%Plot PD
[xData_PD, yData_PD] = get_Data_from(PD_control, L1, L2);
plot(xData_PD, yData_PD, 'g', 'LineWidth', 0.5);
%h = plot(xData_PD(1), yData_PD(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Initial point

%Plot PID
[xData_PID, yData_PID] = get_Data_from(PID_control, L1, L2);
plot(xData_PID, yData_PID, 'k', 'LineWidth', 0.5);
%h = plot(xData(1), yData(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Initial point

% Plot reference signal
[refTh1, refTh2] = Import_refs('square');
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'r', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

%Graph settings
legend('PD Control', 'PID Control','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Simulated Response of Different Control Methods');
grid on;
axis equal;

