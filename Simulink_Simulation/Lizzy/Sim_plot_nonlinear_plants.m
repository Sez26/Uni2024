% Create figure
clear figure;
figure;
hold on;

function [xData, yData] = get_Data_from(workspace_name, L1, L2)
    theta1Signal = workspace_name.get('th1'); % Get 'theta1' signal
    theta2Signal = workspace_name.get('th2'); % Get 'theta2' signal
    theta1Timeseries = theta1Signal.Values;
    theta2Timeseries = theta2Signal.Values;
    th1Data = theta1Timeseries.Data;
    th2Data = theta2Timeseries.Data;
    xData = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData = (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
end

%Plot Plant_0
[xData_0, yData_0] = get_Data_from(Plant_0, L1, L2);
plot(xData_0, yData_0, 'c', 'LineWidth', 0.5);
h = plot(xData_0(1), yData_0(1), 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c'); % Initial point

%Plot Plant_2_1
% [xData_2_1, yData_2_1] = get_Data_from(PI_control, L1, L2);
% plot(xData_2_1, yData_2_1, 'c', 'LineWidth', 0.5);
% h = plot(xData_2_1(1), yData_2_1(1), 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c'); % Initial point

%Plot Plant_3
[xData_3, yData_3] = get_Data_from(Plant_3, L1, L2);
plot(xData_3, yData_3, 'k', 'LineWidth', 0.5);
%h = plot(xData_PD(1), yData_PD(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Initial point

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'r', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

%Graph settings
legend('Plant_0', 'Start point','Plant 3');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Simulated Response of Different Control Methods');
grid on;
axis equal;

