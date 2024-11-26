function [xData_th1, yData_th2] = get_Data_from(workspace_name, L1, L2, nonlinear)
    if nonlinear
        th1Data = workspace_name.get('th1').Values.Data(:,1);
        th2Data = workspace_name.get('th1').Values.Data(:,2);
    else
        th1Data = workspace_name.get('th1').Values.Data;
        th2Data = workspace_name.get('th2').Values.Data;
    end
    
    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2 = (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
end

newcolors = [0.83 0.14 0.14
             1.00 0.54 0.00
             0.47 0.25 0.80
             0.25 0.80 0.54];
         
colororder(newcolors)

% Create figure
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

%Plot PID_control
[xData_PID, yData_PID] = get_Data_from(PID_control, L1, L2, false);
plot(xData_PID(1), yData_PID(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', [0, 0.4470, 0.7410]); % Initial point
plot(xData_PID, yData_PID,'Color', [0, 0.4470, 0.7410],'LineWidth', 0.5);

%Plot Plant_0
[xData_0, yData_0] = get_Data_from(Plant_0, L1, L2, false);
plot(xData_0, yData_0, 'Color',[0.8500 0.3250 0.0980],'LineWidth', 0.5);

%Plot Plant_2_1
[xData_2_1, yData_2_1] = get_Data_from(Plant_2_1, L1, L2, true);
plot(xData_2_1, yData_2_1, 'Color', [0.9290 0.6940 0.1250],'LineWidth', 0.5);

%Plot Plant_3
% [xData_3, yData_3] = get_Data_from(Plant_3, L1, L2);
% plot(xData_3, yData_3, 'LineWidth', 0.5);

%Plot PID_lowpass
[xData_PID_backlash, yData_PID_backlash] = get_Data_from(PID_lowpass, L1, L2, false);
plot(xData_PID_backlash, yData_PID_backlash,'LineWidth', 0.5);

% Plot reference signal
[refTh1, refTh2] = Import_refs('circle');
xData_ref = (L1*cos(deg2rad(refTh1)) + L2*cos(deg2rad(refTh2)))*1000;
yData_ref = (L1*sin(deg2rad(refTh1)) + L2*sin(deg2rad(refTh2)))*1000;
plot(xData_ref, yData_ref, 'k', 'LineWidth', 0.5);
xlim([min(xData_ref)-25, max(xData_ref)+25]);
ylim([min(yData_ref)-25, max(yData_ref)+25]);

%Graph settings
legend('Start point','Linear model','Nonlinear model','State feedback control','backlash','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Simulated Response of Different Control Methods');
grid on;
axis equal;
set(gca,'fontsize', 14) 

