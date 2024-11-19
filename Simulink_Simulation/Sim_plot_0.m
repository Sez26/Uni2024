% Extract data from timeseries objects
xData = x.Data;
yData = y.Data;
timeData = x.Time; % Assuming both timeseries have the same time vector

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