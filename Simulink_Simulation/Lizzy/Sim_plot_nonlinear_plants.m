function [xData_th1, yData_th2] = get_Data_from(workspace_name, L1, L2, nonlinear)
    if nonlinear
        th1Data = workspace_name.get('th1').Values.Data(:,1);
        th2Data = workspace_name.get('th1').Values.Data(:,2);
    else
        th1Data = workspace_name.get('th1').Values.Data;
        th2Data = workspace_name.get('th2').Values.Data;
    end
    
    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2= (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
    
    % xData_th1 = -yData_th2_before;
    % yData_th2 = -xData_th1_before;
end

newcolors = [0.83 0.14 0.14
             1.00 0.54 0.00
             0.47 0.25 0.80
             0.25 0.80 0.54];
         
colororder(newcolors)

% Create figure
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;


%Plot 1
[xData_1, yData_1] = get_Data_from(PID_Izzy, L1, L2, false);
% plot(xData_1(1), yData_1(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', [0, 0.4470, 0.7410]); % Initial point
plot(xData_1(150:end), yData_1(150:end),'Color', [0, 0.4470, 0.7410],'LineWidth', 0.5);

%Plot 2
[xData_2, yData_2] = get_Data_from(PD_control, L1, L2, false);
plot(xData_2(100:end), yData_2(100:end), 'Color',[0.8500 0.3250 0.0980],'LineWidth', 0.5);

[xData_2, yData_2] = get_Data_from(PID_18, L1, L2, false);
plot(xData_2(100:end), yData_2(100:end), 'Color',[0.8500 0.3250 0.0980],'LineWidth', 0.5);

%Plot 3
[xData_3, yData_3] = get_Data_from(PID_19, L1, L2, false);
plot(xData_3(100:end), yData_3(100:end), 'Color', [0.9290 0.6940 0.1250],'LineWidth', 0.5);

% %Plot 4
% [xData_4, yData_4] = get_Data_from(PID_4, L1, L2,false);
% plot(xData_4, yData_4, 'LineWidth', 0.5);
% 
% % %Plot 5
% [xData_5, yData_5] = get_Data_from(PID_5, L1, L2, false);
% plot(xData_5, yData_5,'LineWidth', 1.5);
% 
% % %Plot 6
% [xData_6, yData_6] = get_Data_from(PID_6, L1, L2, false);
% plot(xData_6, yData_6,'LineWidth', 1.5);


% Plot reference signal
[refTh1, refTh2] = Import_refs('square');
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'k', 'LineWidth', 1.5);
% xlim([min(xData_ref)-25, max(xData_ref)+25]);
% ylim([min(yData_ref)-25, max(yData_ref)+25]);

%Graph settings
legend(); %'Linear model','Nonlinear model','State feedback control','backlash','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Simulated Response of Different Control Methods');
grid on;
axis equal;
set(gca,'fontsize', 14) 

