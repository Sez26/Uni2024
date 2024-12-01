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

function plot_shape(data, L1, L2, start)
    [xData, yData] = get_Data_from(data, L1, L2, false);
    plot(xData(start:end), yData(start:end),'LineWidth', 1.5);
end

% Create figure
clear figure;
figure('Renderer', 'painters', 'Position', [10 10 550 500]);
hold on;

% %Plot start point
% plot(xData_1(1), yData_1(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', [0, 0.4470, 0.7410]); % Initial point

% Plot image -------------------------------
img = imread('bumpcircle.jpg');
grayImg = rgb2gray(img);
threshold = 50;  % Adjust if needed
blackPixels = grayImg < threshold;  % Logical mask of black pixels
[row, col] = find(blackPixels);

%transformations
row = (row*0.2) +0; col = (col*0.2) +0; %scale down, move L/R
scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
        'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

%rotation constants
%x_ref = xData(1); y_ref = yData(1);
x_ref = 75; y_ref = 90;
theta = deg2rad(80);
R = [cos(theta), -sin(theta); 
     sin(theta),  cos(theta)];

% Plot reference signal ------------------
[refTh1, refTh2] = import_refs('circle');
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
xData_ref = xData_ref-50; yData_ref = yData_ref+85;
translatedPoints = R*[xData_ref - x_ref; yData_ref - y_ref];
xData_ref = translatedPoints(1, :) + x_ref; yData_ref = translatedPoints(2, :) + y_ref;
plot(xData_ref-5, yData_ref+3, 'k', 'LineWidth', 1.5);
xlim([min(xData_ref)-15, max(xData_ref)+25]);
ylim([min(yData_ref)-15, max(yData_ref)+25]);


%Plot model response
[xData, yData] = get_Data_from(PID_circle_backlash, L1, L2, false);
xData = (xData-50)'; yData = (yData+85)';
translatedPoints = R*[xData - x_ref; yData - y_ref];
xData = translatedPoints(1, :) + x_ref; yData = translatedPoints(2, :) + y_ref;
start = 240; 
plot(xData(start:end)-7, yData(start:end),'r','LineWidth', 1.5);



%Graph settings ----------------------------
legend('Real Response','Target shape','PID Model Response'); %'Linear model','Nonlinear model','State feedback control','backlash','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
grid on;
axis equal;
set(gca,'fontsize', 14) 

