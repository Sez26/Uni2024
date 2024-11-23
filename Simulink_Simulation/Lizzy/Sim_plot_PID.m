
% Create figure
clear figure;
figure;
hold on;

%Plot PD
% Get the signal objects for theta1 and theta2
theta1Signal_PD = PD_control.get('theta1'); % Get 'theta1' signal
theta2Signal_PD = PD_control.get('theta2'); % Get 'theta2' signal
theta1Timeseries_PD = theta1Signal_PD.Values;
theta2Timeseries_PD = theta2Signal_PD.Values;
th1Data_PD = theta1Timeseries_PD.Data;
th2Data_PD = theta2Timeseries_PD.Data;

xData_PD = (L1*cos(deg2rad(th1Data_PD)) + L2*cos(deg2rad(th2Data_PD)))*1000;
yData_PD = (L1*sin(deg2rad(th1Data_PD)) + L2*sin(deg2rad(th2Data_PD)))*1000;
plot(xData_PD, yData_PD, 'r', 'LineWidth', 0.5);
h = plot(xData_PD(1), yData_PD(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Initial point


%Plot PID
% Get the signal objects for theta1 and theta2
theta1Signal = base.get('theta1'); % Get 'theta1' signal
theta2Signal = base.get('theta2'); % Get 'theta2' signal
theta1Timeseries = theta1Signal.Values;
theta2Timeseries = theta2Signal.Values;
th1Data = theta1Timeseries.Data;
th2Data = theta2Timeseries.Data;
time1 = theta1Timeseries.Time;
time2 = theta2Timeseries.Time;

% xData = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
% yData = (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
% plot(xData, yData, 'k', 'LineWidth', 0.5);
% h = plot(xData(1), yData(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Initial point
% 
% %Plot reference signal
% xData_ref = (L1*cos(deg2rad(refTh1.th1(1,:))) + L2*cos(deg2rad(refTh2.th2(1,:))))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1.th1(1,:))) + L2*sin(deg2rad(refTh2.th2(1,:))))*1000;
% plot(xData, yData, 'b--', 'LineWidth', 0.5);
% 
% %Plot square
% squareSide = 86; % Side length in mm
% cornerPoint = [77, -43]; % Bottom-left corner of the square (adjust as needed)
% xSquare = cornerPoint(1) + [0, squareSide, squareSide, 0, 0];
% ySquare = cornerPoint(2) + [0, 0, squareSide, squareSide, 0];
% plot(xSquare, ySquare, 'r-', 'LineWidth', 0.5); % Red solid line for the square


legend('PD control', 'Start point', 'Target square','Reference signal')
xlabel('X (mm)');
ylabel('Y (mm)');
title('Simulated Response of Different Control Methods');
grid on;
axis equal;
xlim([min(xData)-50, max(xData)+50]);
ylim([min(yData)-50, max(yData)+50]);


