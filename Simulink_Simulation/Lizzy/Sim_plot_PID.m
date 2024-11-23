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
    time1 = theta1Timeseries.Time; %unused
    time2 = theta2Timeseries.Time; %unused
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

%Plot square
squareSide = 86; % Side length in mm
cornerPoint = [77, -43]; % Bottom-left corner of the square (adjust as needed)
xSquare = cornerPoint(1) + [0, squareSide, squareSide, 0, 0];
ySquare = cornerPoint(2) + [0, 0, squareSide, squareSide, 0];
plot(xSquare, ySquare, 'r', 'LineWidth', 0.5);

%Plot triangle
% triangleSide = 94; % Side length in mm
% startPoint = [50, -40]; % Bottom-left corner of the triangle (adjust as needed)
% xTriangle = [0, triangleSide, triangleSide / 2, 0] + startPoint(1); % x-coordinates
% yTriangle = [0, 0, sqrt(triangleSide^2 - (triangleSide / 2)^2), 0] + startPoint(2); % y-coordinates
% plot(xTriangle, yTriangle, 'r', 'LineWidth', 0.5);

%Plot circle
% circleRadius = 44;
% circleCenter = [100, -20];
% theta = linspace(0, 2*pi, 100); % Angle values from 0 to 2*pi
% xCircle = circleCenter(1) + circleRadius * cos(theta); % x-coordinates
% yCircle = circleCenter(2) + circleRadius * sin(theta); % y-coordinates
% plot(xCircle, yCircle, 'r', 'LineWidth', 0.5);

legend('PD Control', 'PID Control','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Simulated Response of Different Control Methods');
grid on;
axis equal;
xlim([min(xData)-50, max(xData)+50]);
ylim([min(yData)-50, max(yData)+50]);

figure;
% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1.th1(1,:))) + L2*cos(deg2rad(refTh2.th2(1,:))))*1000;
yData_ref = (L1*sin(deg2rad(refTh1.th1(1,:))) + L2*sin(deg2rad(refTh2.th2(1,:))))*1000;
plot(xData_ref, yData_ref, 'b', 'LineWidth', 0.5);
xlim([min(xData)-50, max(xData)+50]);
ylim([min(yData)-50, max(yData)+50]);