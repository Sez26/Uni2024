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
img = imread('square1.jpg');
grayImg = rgb2gray(img);
threshold = 50;  % Adjust if needed
blackPixels = grayImg < threshold;  % Logical mask of black pixels
[row, col] = find(blackPixels);

%transformations
row = (row*0.14) +0; col = (col*0.14) +0; %scale down, move L/R
scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
        'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

% Plot reference signal ------------------
[refTh1, refTh2] = import_refs('square');
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;

%for ref overlay:
xData_ref = xData_ref-64; yData_ref = yData_ref+82;
plot(xData_ref, yData_ref, 'r', 'LineWidth', 1.5);

xlim([min(xData_ref)-15, max(xData_ref)+25]);
ylim([min(yData_ref)-15, max(yData_ref)+25]);

%Plot model response
[xData, yData] = get_Data_from(PID_square_backlash, L1, L2, false);
xData = (xData-66); yData = (yData+80);

start = 1000; 
plot(xData(start:end), yData(start:end),'k','LineWidth', 1.5);




%Graph settings ----------------------------
legend('Real Response','PID Model Response','Target shape'); %'Linear model','Nonlinear model','State feedback control','backlash','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
grid on;
axis equal;
set(gca,'fontsize', 14) 

