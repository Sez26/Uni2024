%% script to plot motor id figure - (x is -12 to +12) - (y is -4.14 to +4.14 (min to max torque) assuming completely linear relation between velocity and torque)

% import figure
motor1data = readmatrix("C:\Users\Izzy Popiolek\Documents\GitHub\Uni2024_MVNLC\arduino_control\system_id\resultsmotor1.csv");
motor2data = readmatrix("C:\Users\Izzy Popiolek\Documents\GitHub\Uni2024_MVNLC\arduino_control\system_id\resultsmotor2.csv");
% motor1velocity = motor1data(:,2);
% motor2velocity = motor2data(:,2);

% Normalize the vector
% m1_torque = (motor1data(:,2) - min(motor1data(:,2))) * (4.14 - -4.14) / (max(motor1data(:,2)) - min(motor1data(:,2))) + -4.14;
% m1_voltage = (motor1data(:,1) - min(motor1data(:,1))) * (12 - -12) / (max(motor1data(:,1)) - min(motor1data(:,1))) + -12;
m2_torque = (motor2data(:,2) - min(motor2data(:,2))) * (4.14 - -4.14) / (max(motor2data(:,2)) - min(motor2data(:,2))) + -4.14;
m2_voltage = (motor2data(:,1) - min(motor2data(:,1))) * (12 - -12) / (max(motor2data(:,1)) - min(motor2data(:,1))) + -12;

% add linear fit line in certain sections
p = polyfit(motor2data(:,1), m2_torque, 1);
y_fit = polyval(p, motor2data(:,1));


% plot together
figure;
% plot(m2_voltage(:,1), m1_normalized, '-k', 'LineWidth', 1.5);
% hold on
plot(m2_voltage(:), m2_torque, '-k', 'LineWidth', 1.5);
hold on;
% plot(m2_voltage(:,1), y_fit, '-b', 'LineWidth', 1.5);
% hold on;
plot([m2_voltage(1), m2_voltage(209)], [m2_torque(1), m2_torque(209)],'-r', 'LineWidth', 1.5);
hold on;
plot([m2_voltage(303), m2_voltage(512)], [m2_torque(303), m2_torque(512)],'-r', 'LineWidth', 1.5);
hold on;
plot([m2_voltage(209), m2_voltage(303)], [0, 0],'-r', 'LineWidth', 1.5);
xlabel('Voltage (V)');
ylabel('Torque (Nm)');
% title('Dead Zone Modelling');
legend('Raw Data','Model Input', 'Location', 'northwest')
grid on;