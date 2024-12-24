% Example data points
x = [1, 2, 3, 4, 5];
y = [1, 1, 2, 2, 3];

% Rotation angle in degrees
theta_deg = 30;  
theta = deg2rad(theta_deg);  % Convert to radians

% Define the rotation matrix
R = [cos(theta), -sin(theta); 
     sin(theta),  cos(theta)];

% Combine x and y into a matrix
points = [x; y];

% Apply the rotation
rotatedPoints = R * points;

% Extract rotated x and y coordinates
x_rotated = rotatedPoints(1, :);
y_rotated = rotatedPoints(2, :);

% Plot the original and rotated points
figure;
plot(x, y, 'bo-', 'LineWidth', 2); hold on;  % Original points
plot(x_rotated, y_rotated, 'ro-', 'LineWidth', 2);  % Rotated points
legend('Original', 'Rotated');
xlabel('X-axis');
ylabel('Y-axis');
title(['Rotation by ', num2str(theta_deg), '°']);
axis equal;