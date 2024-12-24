% Create figure
close all;
clear figure;
figure;
hold on;

%%

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'r', 'LineWidth', 2);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

% Plot reference signal (motor 1 +100 encoder counts) 17 degrees
xData_ref = (L1*cos(deg2rad(refTh1+5)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+5)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'b', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

% Plot reference signal (motor 1 -100 encoder counts)
xData_ref = (L1*cos(deg2rad(refTh1-5)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-5)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'g', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

% Plot reference signal (motor 2 +100 encoder counts)
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2+5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2+5)))*1000;
plot(xData_ref, yData_ref, 'k', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

% Plot reference signal (motor 2 -100 encoder counts)
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2-5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2-5)))*1000;
plot(xData_ref, yData_ref, 'y', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);

% % Plot reference signal (motor 2 -100 encoder counts)
% xData_ref = (L1*cos(deg2rad(refTh1+5)) + L2*cos(deg2rad(refTh2-5)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1+5)) + L2*sin(deg2rad(refTh2-5)))*1000;
% plot(xData_ref, yData_ref, 'r', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);

%Graph settings
legend('Target shape', 'motor 1 left by 5 degrees', 'motor 1 right by 5 degrees', 'motor 2 left by 5 degrees', 'motor 2 right by 5 degrees', 'combination');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Impact of Backlash and Human Error in Upright Starting Position');
grid on;
axis equal;


% degree to encoder count calculator. 
% determine the approximate average backlash error from the system in
% upright position

% plot the bounds of this 
% do all combinations of differences fit within?

% show physical testing results as well

%% - vertical straight average errors with bounds
% Create figure
close all;
clear figure;
figure;
hold on;

allx = zeros(1000,5);
ally = zeros(1000,5);


% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'k', 'LineWidth', 2);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,1) = xData_ref;
ally(:,1) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1-2.5)) + L2*cos(deg2rad(refTh2-3.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-2.5)) + L2*sin(deg2rad(refTh2-3.6)))*1000;
plot(xData_ref, yData_ref, 'b', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,2) = xData_ref;
ally(:,2) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1+2.5)) + L2*cos(deg2rad(refTh2+3.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+2.5)) + L2*sin(deg2rad(refTh2+3.6)))*1000;
plot(xData_ref, yData_ref, 'c', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,3) = xData_ref;
ally(:,3) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1+2.5)) + L2*cos(deg2rad(refTh2-3.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+2.5)) + L2*sin(deg2rad(refTh2-3.6)))*1000;
plot(xData_ref, yData_ref, 'm', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,4) = xData_ref;
ally(:,4) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1-2.5)) + L2*cos(deg2rad(refTh2+3.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-2.5)) + L2*sin(deg2rad(refTh2+3.6)))*1000;
plot(xData_ref, yData_ref, 'g', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,5) = xData_ref;
ally(:,5) = yData_ref;

% calculate minimum and maximum bounds in a square shape and plot in dotted
% red line
x1=min(allx, [], 'all');
x2=max(allx, [], 'all');
y1=min(ally, [], 'all');
y2=max(ally, [], 'all');
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
bigarea = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
plot(x, y, '--r', 'LineWidth', 0.5);
hold on;


nx1=86.16;
nx2=157.51;
ny1=-34.49;
ny2=33.8;
x = [nx1, nx2, nx2, nx1, nx1];
y = [ny1, ny1, ny2, ny2, ny1];
plot(x, y, '--r', 'LineWidth', 0.5);
hold on;

% bounding box area
% Calculate the area using the Shoelace formula
smallarea = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
boxarea = (bigarea - smallarea) / 100;

% Display the result
fprintf('The area of the square is: %.2f\n', boxarea);

% find and print the mm difference between the target shape and the bounds

% for each of the 1000 points find the corresponding furthest point of each
% iteration and measure the distance away. add togther these distances and
% divide by 1000 to find the average. compare the crunch average and find
% the percentage reduction.


%Graph settings
% legend('Target shape', 'motor 1 - 2.5, motor 2 - 3.6', 'motor 1 + 2.5, motor 2 + 3.6');
xlabel('X (mm)');
ylabel('Y (mm)');
% title('Impact of Backlash and Human Error in Upright Starting Position');
grid on;
axis equal;


%% - vertical crunch average errors with bounds
% Create figure
close all;
clear figure;
figure;
hold on;

allx = zeros(1000,5);
ally = zeros(1000,5);


% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'k', 'LineWidth', 2);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,1) = xData_ref;
ally(:,1) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2-1.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2-1.6)))*1000;
plot(xData_ref, yData_ref, 'b', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,2) = xData_ref;
ally(:,2) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2+1.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2+1.6)))*1000;
plot(xData_ref, yData_ref, 'c', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,3) = xData_ref;
ally(:,3) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2-1.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2-1.6)))*1000;
plot(xData_ref, yData_ref, 'm', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,4) = xData_ref;
ally(:,4) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2+1.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2+1.6)))*1000;
plot(xData_ref, yData_ref, 'g', 'LineWidth', 0.5);
xlim([min(xData_ref)-50, max(xData_ref)+50]);
ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,5) = xData_ref;
ally(:,5) = yData_ref;

% calculate minimum and maximum bounds in a square shape and plot in dotted
% red line
x1=min(allx, [], 'all');
x2=max(allx, [], 'all');
y1=min(ally, [], 'all');
y2=max(ally, [], 'all');
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
bigarea = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
plot(x, y, '--r', 'LineWidth', 0.5);
hold on;


nx1=81.67;
nx2=160.3;
ny1=-38.07;
ny2=38.82;
x = [nx1, nx2, nx2, nx1, nx1];
y = [ny1, ny1, ny2, ny2, ny1];
plot(x, y, '--r', 'LineWidth', 0.5);
hold on;

% bounding box area
% Calculate the area using the Shoelace formula
smallarea = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
boxarea = (bigarea - smallarea) / 100;

% Display the result
fprintf('The area of the square is: %.2f\n', boxarea);

% find and print the mm difference between the target shape and the bounds

% for each of the 1000 points find the corresponding furthest point of each
% iteration and measure the distance away. add togther these distances and
% divide by 1000 to find the average. compare the crunch average and find
% the percentage reduction.


%Graph settings
%legend('Target shape', 'motor 1 - 2.5, motor 2 - 3.6', 'motor 1 + 2.5, motor 2 + 3.6');
xlabel('X (mm)');
ylabel('Y (mm)');
% title('Impact of Backlash and Human Error in Upright Folded Position');
grid on;
axis equal;

%% bounding box difference calculations

upright = 54.90;
crunch = 27.99;

% Calculate percentage decrease
percentage_decrease = ((upright - crunch) / upright) * 100;

% Display the result
fprintf('The percentage decrease from upright to crunch is: %.2f%%\n', percentage_decrease);