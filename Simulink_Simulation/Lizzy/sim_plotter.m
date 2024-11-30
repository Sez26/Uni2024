function [xData_th1, yData_th2] = get_Data_from(data, L1, L2, flag)
    if string(flag) == "thetas"
        th1Data = data.get('th1').Values.Data;
        th2Data = data.get('th2').Values.Data;
    else
        th1Data = data.get('ref_th1').Values.Data;
        th2Data = data.get('ref_th2').Values.Data; 
    end

    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2= (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
end

function plot_shape(data, L1, L2, start)
    %plot response
    [xData, yData] = get_Data_from(data, L1, L2,"thetas");
    % Initial point
    % plot(xData_1(1), yData_1(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', [0, 0.4470, 0.7410]); 
    plot(xData(start:end), yData(start:end),'b','LineWidth', 1.5);
    
    %plot references
    [xData, yData] = get_Data_from(data, L1, L2,"response");
    plot(xData(start:end), yData(start:end),'r','LineWidth', 1.5);
end

% Create figure
clear figure;
figure('Renderer', 'painters', 'Position', [10 10 550 500]);
hold on;

% % Plot image -------------------------------
% img = imread('square1.jpg');
% grayImg = rgb2gray(img);
% threshold = 50;  % Adjust if needed
% blackPixels = grayImg < threshold;  % Logical mask of black pixels
% [row, col] = find(blackPixels);
% 
% %transformations
% row = (row*0.14) +0; col = (col*0.14) +0; %scale down, move L/R
% scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
%         'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

start = 1; 
plot_shape(PID_bspline, L1, L2, start);

%Graph settings ----------------------------
legend('Response','Target shape'); %'Linear model','Nonlinear model','State feedback control','backlash','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
grid on;
axis equal;
set(gca,'fontsize', 14) 

