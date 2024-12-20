% Create figure
close all;
clear figure;
figure;
hold on;

[refTh1, refTh2] = Import_refs("square");

% %%
% 
% % Plot reference signal
% xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
% plot(xData_ref, yData_ref, 'r', 'LineWidth', 2);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
% 
% % Plot reference signal (motor 1 +100 encoder counts) 17 degrees
% xData_ref = (L1*cos(deg2rad(refTh1+5)) + L2*cos(deg2rad(refTh2)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1+5)) + L2*sin(deg2rad(refTh2)))*1000;
% plot(xData_ref, yData_ref, 'b', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
% 
% % Plot reference signal (motor 1 -100 encoder counts)
% xData_ref = (L1*cos(deg2rad(refTh1-5)) + L2*cos(deg2rad(refTh2)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1-5)) + L2*sin(deg2rad(refTh2)))*1000;
% plot(xData_ref, yData_ref, 'g', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
% 
% % Plot reference signal (motor 2 +100 encoder counts)
% xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2+5)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2+5)))*1000;
% plot(xData_ref, yData_ref, 'k', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
% 
% % Plot reference signal (motor 2 -100 encoder counts)
% xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2-5)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2-5)))*1000;
% plot(xData_ref, yData_ref, 'y', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
% 
% % % Plot reference signal (motor 2 -100 encoder counts)
% % xData_ref = (L1*cos(deg2rad(refTh1+5)) + L2*cos(deg2rad(refTh2-5)))*1000;
% % yData_ref = (L1*sin(deg2rad(refTh1+5)) + L2*sin(deg2rad(refTh2-5)))*1000;
% % plot(xData_ref, yData_ref, 'r', 'LineWidth', 0.5);
% % xlim([min(xData_ref)-50, max(xData_ref)+50]);
% % ylim([min(yData_ref)-50, max(yData_ref)+50]);
% 
% %Graph settings
% legend('Target shape', 'motor 1 left by 5 degrees', 'motor 1 right by 5 degrees', 'motor 2 left by 5 degrees', 'motor 2 right by 5 degrees', 'combination');
% xlabel('X (mm)');
% ylabel('Y (mm)');
% title('Impact of Backlash and Human Error in Upright Starting Position');
% grid on;
% axis equal;
% 
% 
% % degree to encoder count calculator. 
% % determine the approximate average backlash error from the system in
% % upright position
% 
% % plot the bounds of this 
% % do all combinations of differences fit within?
% 
% % show physical testing results as well

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
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,4) = xData_ref;
ally(:,4) = yData_ref;

% Plot reference signal
xData_ref = (L1*cos(deg2rad(refTh1-2.5)) + L2*cos(deg2rad(refTh2+3.6)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-2.5)) + L2*sin(deg2rad(refTh2+3.6)))*1000;
plot(xData_ref, yData_ref, 'g', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,5) = xData_ref;
ally(:,5) = yData_ref;

% calculate minimum and maximum bounds in a square shape and plot in dotted
% red line
x1=min(allx, [], 'all');
x2=max(allx, [], 'all');
y1=min(ally, [], 'all');
y2=max(ally, [], 'all');
disp(x1);
disp(x2);
disp(y1);
disp(y2);
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
set(gcf, 'Position', [100, 100, 400, 400]);
hold on;

% Plot image --------------------------------------------------------------
img = imread('square2.jpg');
img = (flipdim(img , 1));
grayImg = rgb2gray(img);
threshold = 50;  % Adjust if needed
blackPixels = grayImg < threshold;  % Logical mask of black pixels
[row, col] = find(blackPixels);

% % %transformation for sq2
row = (row*0.263) +0; col = (col*0.263) +0; %scale down, move L/R
xoffset = 55;
yoffset = -141;

% for both
s1 = scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
        'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

allx = zeros(1000,5);
ally = zeros(1000,5);


% Plot reference signal----------------------------------------------
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
r1 = plot(xData_ref-xoffset, yData_ref-yoffset, 'k', 'LineWidth', 0.1);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,1) = xData_ref;
ally(:,1) = yData_ref;

% weird signals too --------------------------------------------------
xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2-1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2-1.5)))*1000;
% plot(xData_ref, yData_ref, 'b', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,2) = xData_ref;
ally(:,2) = yData_ref;

xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
% plot(xData_ref, yData_ref, 'c', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,3) = xData_ref;
ally(:,3) = yData_ref;

xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2-1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2-1.5)))*1000;
% plot(xData_ref, yData_ref, 'm', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,4) = xData_ref;
ally(:,4) = yData_ref;

xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
% plot(xData_ref, yData_ref, 'g', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,5) = xData_ref;
ally(:,5) = yData_ref;

% calculate minimum and maximum bounds in a square shape and plot in dotted
% red line
% % inner square -------------------------------------------------
x1=min(allx, [], 'all');
x2=max(allx, [], 'all');
y1=min(ally, [], 'all');
y2=max(ally, [], 'all');
disp('x1=')
disp(x1);
disp('x2=')
disp(x2);
disp('y1=')
disp(y1);
disp('y2=')
disp(y2);
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
bigarea = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
b1 = plot(x-xoffset, y-yoffset, '--r', 'LineWidth', 1);
hold on;

nx1=81.67;
nx2=160.3;
ny1=-38.81;
ny2=38.82;
x = [nx1, nx2, nx2, nx1, nx1];
y = [ny1, ny1, ny2, ny2, ny1];
plot(x-xoffset, y-yoffset, '--r', 'LineWidth', 1);
hold on;
xlim([0 100]);
ylim([80 200]);
axis equal;
axis off;
grid off;
legend([s1 r1 b1], {'Real Shape', 'Reference Shape', 'Backlash Error Bounds'},'Position', [0.44, 0.5, 0.2, 0.1]);
exportgraphics(gca, 'figure.png', 'BackgroundColor', 'none', 'ContentType', 'image');
%%

% %triangle----------------------------------------------------------------
% % Define the coordinates for the triangle
% % Side length of the equilateral triangle
% sideLength = 79;  % You can adjust this value
% 
% % First vertex (starting point)
% nx1 = 87.9;
% ny1 = -45.6;
% 
% % Calculate the other two vertices
% % Vertex 2 is horizontally to the right of vertex 1
% nx2 = nx1 + sideLength;
% ny2 = ny1;
% 
% % Vertex 3 is above the center of the line formed by vertices 1 and 2
% height = sqrt(3) / 2 * sideLength;  % height of an equilateral triangle
% nx3 = (nx1 + nx2) / 2;  % x-coordinate of the third vertex (centered)
% ny3 = ny1 + height;  % y-coordinate of the third vertex
% 
% % Coordinates of the triangle
% x = [nx1, nx2, nx3, nx1];  % x-coordinates of the vertices
% y = [ny1, ny2, ny3, ny1];  % y-coordinates of the vertices
% 
% % Plot the equilateral triangle
% plot(x, y, '--r', 'LineWidth', 0.5);
% axis equal;  % Equal scaling of x and y axes
% grid on;
% hold on;
% 
% % Define the coordinates for the triangle
% % Side length of the equilateral triangle
% sideLength = 108;  % You can adjust this value
% 
% % First vertex (starting point)
% nx1 = 72.3;
% ny1 = -54.5;
% 
% % Calculate the other two vertices
% % Vertex 2 is horizontally to the right of vertex 1
% nx2 = nx1 + sideLength;
% ny2 = ny1;
% 
% % Vertex 3 is above the center of the line formed by vertices 1 and 2
% height = sqrt(3) / 2 * sideLength;  % height of an equilateral triangle
% nx3 = (nx1 + nx2) / 2;  % x-coordinate of the third vertex (centered)
% ny3 = ny1 + height;  % y-coordinate of the third vertex
% 
% % Coordinates of the triangle
% x = [nx1, nx2, nx3, nx1];  % x-coordinates of the vertices
% y = [ny1, ny2, ny3, ny1];  % y-coordinates of the vertices
% 
% % Plot the equilateral triangle
% plot(x, y, '--r', 'LineWidth', 0.5);
% axis equal;  % Equal scaling of x and y axes
% grid on;
% hold on;

% % circle------------------------------------------------------------------
% r = 39.5;  % Radius of the circle
% x_center = 120;  % x-coordinate of the center
% y_center = 0;  % y-coordinate of the center
% 
% % Generate theta values (0 to 2*pi for a full circle)
% theta = linspace(0, 2*pi, 100);  % 100 points to make the circle smooth
% 
% % Parametric equations for the circle
% x = r * cos(theta) + x_center;  % x-coordinates
% y = r * sin(theta) + y_center;  % y-coordinates
% 
% % Plot the circle
% plot(x, y, '--r', 'LineWidth', 0.5);
% axis equal;  % Equal scaling of x and y axes
% grid on;
% hold on;

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
% axis off;

%% bounding box difference calculations

upright = 54.90;
crunch = 27.99;

% Calculate percentage decrease
percentage_decrease = ((upright - crunch) / upright) * 100;

% Display the result
fprintf('The percentage decrease from upright to crunch is: %.2f%%\n', percentage_decrease);

% %%
% A = imread("triangle.jpg");
% image(A)
% % xlim([0 5]);    % x-axis limits
% % ylim([-1 1]);   % y-axis limits
% % xticks(0:1:5);              % Tick positions on the x-axis
% % yticks([-1, 0, 1]);         % Tick positions on the y-axis
% axis equal

%% plotting with real drawings involved - square
% Create figure
close all;
clear figure;
figure;
hold on;

% initialise variables ----------------------------------------------------
allx = zeros(1000,5);
ally = zeros(1000,5);

% Plot image --------------------------------------------------------------
img = imread('square2.jpg');
img = (flipdim(img , 1));
grayImg = rgb2gray(img);
threshold = 50;  % Adjust if needed
blackPixels = grayImg < threshold;  % Logical mask of black pixels
[row, col] = find(blackPixels);

%transformations for sq1
% row = (row*0.14) +0; col = (col*0.14) +0; %scale down, move L/R
% xoffset = 63;
% yoffset = -76.5;

%transformation for sq2
row = (row*0.265) +0; col = (col*0.265) +0; %scale down, move L/R
xoffset = 53;
yoffset = -144.5;

% for both
scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
        'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

% % Plot reference signal ---------------------------------------------------
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
% plot(xData_ref-xoffset, yData_ref-yoffset, 'k', 'LineWidth', 1);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,1) = xData_ref;
ally(:,1) = yData_ref;
 
% Plot backlash impacted signals ------------------
xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2-1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2-1.5)))*1000;
plot(xData_ref(1:500)-xoffset, yData_ref(1:500)-yoffset, 'b', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,2) = xData_ref;
ally(:,2) = yData_ref;

xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
plot(xData_ref(1:500)-xoffset, yData_ref(1:500)-yoffset, 'c', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,3) = xData_ref;
ally(:,3) = yData_ref;

xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
plot(xData_ref(501:1000)-xoffset, yData_ref(501:1000)-yoffset, 'm', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,4) = xData_ref;
ally(:,4) = yData_ref;


xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2-1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2-1.5)))*1000;
plot(xData_ref(501:1000)-xoffset, yData_ref(501:1000)-yoffset, 'r', 'LineWidth', 0.5);
% xlim([min(xData_ref)-50, max(xData_ref)+50]);
% ylim([min(yData_ref)-50, max(yData_ref)+50]);
allx(:,5) = xData_ref;
ally(:,5) = yData_ref;
% 
% % calculate minimum and maximum bounds in a square shape and plot ---------
% x1=min(allx, [], 'all');
% x2=max(allx, [], 'all');
% y1=min(ally, [], 'all');
% y2=max(ally, [], 'all');
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% bigarea = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
% plot(x-xoffset, y-yoffset, '--r', 'LineWidth', 0.5);
% hold on;
% 
% nx1=81.67;
% nx2=160.3;
% ny1=-38.07;
% ny2=38.82;
% x = [nx1, nx2, nx2, nx1, nx1];
% y = [ny1, ny1, ny2, ny2, ny1];
% plot(x-xoffset, y-yoffset, '--r', 'LineWidth', 0.5);
% hold on;

%Graph settings
legend('real shape', 'motor 1- motor2-', 'motor 1- motor 2+', 'motor 1+ motor 2+', 'motor 1+ motor2-');
xlabel('x (mm)');
ylabel('y (mm)');
grid on;
axis equal;