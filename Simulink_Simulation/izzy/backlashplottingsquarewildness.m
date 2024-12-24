[refTh1, refTh2] = Import_refs("square");

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

%% plotting with real drawings involved - square
% Create figure
close all;
clear figure;
figure;
set(gcf, 'Position', [100, 100, 400, 400]);
hold on;

% initialise variables ----------------------------------------------------
allx = zeros(1000,5);
ally = zeros(1000,5);
xoffset = 0;
yoffset = 0;

% Plot image --------------------------------------------------------------
img = imread('square2.jpg');
img = (flipdim(img , 1));
grayImg = rgb2gray(img);
threshold = 50;  % Adjust if needed
blackPixels = grayImg < threshold;  % Logical mask of black pixels
[row, col] = find(blackPixels);

%transformations for sq1
% row = (row*0.14) +0; col = (col*0.14) +0; %scale down, move L/R
xoffset = 63;
yoffset = -76.5;

% % %transformation for sq2
row = (row*0.263) +0; col = (col*0.263) +0; %scale down, move L/R
xoffset = 55;
yoffset = -141;

% for both
s1 = scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
        'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

% % Plot reference signal ---------------------------------------------------
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
t1 = plot(xData_ref-xoffset, yData_ref-yoffset, 'k', 'LineWidth', 0.1);
hold on;
% Plot backlash impacted signals ------------------
linewidth = 2;
xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2-1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2-1.5)))*1000;
h2 = plot(xData_ref(125:330)-xoffset, yData_ref(125:330)-yoffset, 'g', 'LineWidth', linewidth, 'DisplayName','motor 1- motor 2-');
hold on;
% xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
% plot(xData_ref(1:125)-xoffset, yData_ref(1:125)-yoffset, 'c', 'LineWidth', 1.5);
% hold on;
% xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
% plot(xData_ref(250:355)-xoffset, yData_ref(250:355)-yoffset, 'c', 'LineWidth', 1.5);
% hold on;
xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
h5 = plot(xData_ref(675:780)-xoffset, yData_ref(675:780)-yoffset, 'm', 'LineWidth', linewidth);
plot(xData_ref(875:900)-xoffset, yData_ref(875:900)-yoffset, 'm', 'LineWidth', linewidth);
hold on;
% xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2-1.5)))*1000;
% yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2-1.5)))*1000;
% plot(xData_ref(501:590)-xoffset, yData_ref(501:590)-yoffset, 'r', 'LineWidth', 1.5);
% hold on;
xData_ref = (L1*cos(deg2rad(refTh1-1.5)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1-1.5)) + L2*sin(deg2rad(refTh2)))*1000;
h1 = plot(xData_ref(30:110)-xoffset, yData_ref(30:110)-yoffset, 'b', 'LineWidth', 1.5);
hold on;
xData_ref = (L1*cos(deg2rad(refTh1+1.5)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1+1.5)) + L2*sin(deg2rad(refTh2)))*1000;
h4 = plot(xData_ref(800:875)-xoffset, yData_ref(800:875)-yoffset, 'r', 'LineWidth', linewidth);
hold on;
plot(xData_ref(920:1000)-xoffset, yData_ref(920:1000)-yoffset, 'r', 'LineWidth', linewidth);
hold on;
plot(xData_ref(630:670)-xoffset, yData_ref(630:670)-yoffset, 'r', 'LineWidth', linewidth);

xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2+1.5)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2+1.5)))*1000;
h3 = plot(xData_ref(410:610)-xoffset, yData_ref(410:610)-yoffset, 'y', 'LineWidth', linewidth);
hold on;

% Create figure
axis equal;
hold on;
%Graph settings
% legend('real shape', 'target', 'motor 1- motor2-', 'motor 1- motor 2+', 'motor 1+ motor 2+', 'motor 1+ motor 2+', 'motor 1+ motor2-','motor 1+' 'ref', 'modelout');
xlabel('x (mm)');
ylabel('y (mm)');
grid on;
hold on;
xmove = 20;
ymove = -236;
xlim([0 120]);
ylim([80 200]);
% [xData, yData] = get_Data_from(PID_square_backlash,L1,L2,false);
% xData = (xData); yData = (yData);
% start = 1000; 
% plot(xData(start:end)-xmove, yData(start:end)-ymove,'k','LineWidth',1.5);
% hold on;
% [xData, yData] = get_Data_ref(PID_square_backlash,L1,L2,false);
% xData = (xData); yData = (yData);
% start = 1000; 
% plot(xData(start:end)-xmove, yData(start:end)-ymove,'r','LineWidth',1.5);
grid off;
axis off;
legend([s1 t1 h1 h2 h3 h4 h5], {'Real Shape', 'Reference Shape', 'Backlash: m1-', 'Backlash: m1- m2-', 'Backlash: m2+', 'Backlash: m1+', 'Backlash: m1+ m2+'}, 'Position', [0.44, 0.5, 0.2, 0.1]); %  'ref', 'modelout');

%% functions

function [xData_th1, yData_th2] = get_Data_from(data, L1, L2, nonlinear)
    if nonlinear
        th1Data = data.get('th1').Values.Data(:,1);
        th2Data = data.get('th1').Values.Data(:,2);
    else
        th1Data = data.get('th1').Values.Data;
        th2Data = data.get('th2').Values.Data;
    end
    
    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2= (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
    
    %translation 90 degrees
    % xData_th1 = -yData_th2_before;
    % yData_th2 = -xData_th1_before;
end

function [xData_th1, yData_th2] = get_Data_ref(data, L1, L2, nonlinear)
    if nonlinear
        th1Data = data.get('ref_th1').Values.Data(:,1);
        th2Data = data.get('ref_th1').Values.Data(:,2);
    else
        th1Data = data.get('ref_th1').Values.Data;
        th2Data = data.get('ref_th2').Values.Data;
    end
    
    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2= (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
    
    %translation 90 degrees
    % xData_th1 = -yData_th2_before;
    % yData_th2 = -xData_th1_before;
end