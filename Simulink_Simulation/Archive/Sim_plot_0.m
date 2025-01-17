% Extract data from timeseries objects
th1Data = op_th1.Data;
th2Data =  op_th2.Data;
timeData = op_th1.Time; % Assuming both timeseries have the same time vector

xData = L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data));
yData = L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data));

% Create figure
figure;
hold on;
plot(xData, yData, 'k--', 'LineWidth', 0.5); % Optional: plot the trajectory for reference
h = plot(xData(1), yData(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Initial point
xlabel('x');
ylabel('y');
title('Animation of y against x');
grid on;
axis equal;
xlim([min(xData)-0.1, max(xData)+0.1]);
ylim([min(yData)-0.1, max(yData)+0.1]);

% Animation loop
for k = 1:length(timeData)
    set(h, 'XData', xData(k), 'YData', yData(k)); % Update point
    pause(0.05); % Pause to control speed
end