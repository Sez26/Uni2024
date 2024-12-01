%% Statefeedback integral action
Initialise_var_0
C = [1 0 0 0; 0 1 0 0];
D = [0 0;0 0];
E = [1 0 0 0; 0 1 0 0];

% from linearised A and B matrices
% tune controller matrix K

[nx, nu] = size(B); % nx: number of states, nu: number of inputs
[ny, ~] = size(C);  % ny: number of outputs

% Augment system for integral action
n = size(A, 1);   % Number of states
m = size(C, 1);   % Number of outputs
A_aug = [A, zeros(n, m);
         -E, zeros(m, m)];
B_aug = [B; zeros(m, size(B, 2))];

% Controllability check
Ctrb = ctrb(A_aug, B_aug);
if rank(Ctrb) < size(A_aug, 1)
    disp('Augmented system is not controllable.');
else
    disp('Augmented system is controllable.');
end

% Define desired closed-loop poles (tune these as needed)
% desired_poles = [-40+100*1i, -40-100*1i, -20+100*1i, -20-100*1i, -80, -90];
desired_poles = [-zeta * omega_n + omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -zeta * omega_n - omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -5 * zeta * omega_n, -6 * zeta * omega_n, -190, -256];

% Q = diag([100 100 100 100 1000 1000]); % State and integral action weights
% R = eye(nu);                       % Control effort weight
% K = lqr(Ae, Be, Q, R);

% Compute gains using pole placement
K_aug = place(A_aug, B_aug, desired_poles);

% Extract state feedback and integral gains
Kx = K_aug(:, 1:n);   % State feedback gains
Ki = K_aug(:, n+1:end); % Integral gains
% 
% Kx = K(:, 1:n);   % State feedback gains
% Ki = K(:, n+1:end); % Integral gains

% Display the gains
disp('State feedback gains (Kx):');
disp(Kx);
disp('Integral gains (Ki):');
disp(Ki);

% % closed loop
eig([A-B*Kx, -B*Ki; -E, zeros(ny)])

% % Check dimensions
% [nx, nu] = size(B); % nx: number of states, nu: number of inputs
% [ny, ~] = size(C);  % ny: number of outputs
% 
% % Augment the system for integral action
% Ae = [A zeros(nx, ny); -C zeros(ny)];
% Be = [B; zeros(ny, nu)];
% 
% % Desired settling time (Ts) in seconds
% Ts = 0.2; % Specify your desired settling time
% 
% % Calculate desired pole locations
% zeta = 0.2; % Damping ratio
% omega_n = 4 / (zeta * Ts); % Natural frequency
% % Assume four complex-conjugate poles for demonstration
% desired_poles = [-zeta * omega_n + omega_n * sqrt(1 - zeta^2) * 1i, ...
%                  -zeta * omega_n - omega_n * sqrt(1 - zeta^2) * 1i, ...
%                  -5 * zeta * omega_n, -6 * zeta * omega_n, -10 , -12];
% desired_poles = [-2, -3, -4, -5, -6, -7];
% 
% % Check controllability
% ctrb_matrix = ctrb(A, B);
% if rank(ctrb_matrix) < size(A, 1)
%     error('The system is not controllable and cannot be stabilized.');
% end
% 
% % Compute state-feedback gain K using pole placement
% % Since B has multiple columns, use the 'place' function for MIMO
% % K = place(Ae, Be, desired_poles);
% 
% % % Design state feedback with integral action using LQR
% % Q = diag([10 10 10 10 1 1]); % State and integral action weights
% % R = eye(nu);                       % Control effort weight
% % K = lqr(Ae, Be, Q, R);
% % Extract gains
% Kx = K(:, 1:nx);    % State feedback gains (size nu x nx, i.e., 2x4)
% Ki = K(:, nx+1:end); % Integral action gains (size nu x ny, i.e., 2x2)
% 
% % Display the gains
% disp('State feedback gains (Kx):');
% disp(Kx);
% 
% disp('Integral action gains (Ki):');
% disp(Ki);
% 
% % closed loop
% eig([A-B*Kx, B*Ki; -E, zeros(ny)])

% % Closed-loop system matrices
% Acl = [A -B*Kx; -C zeros(ny, 4)]; % Acl is now 6x6
% Bcl = [zeros(nx, ny); eye(ny)];
% Ccl = [C zeros(ny, ny)];
% Dcl = zeros(ny);
% 
% % Simulate the closed-loop response
% t = 0:0.01:10;                  % Time vector
% r = [ones(size(t)); zeros(size(t))]; % Reference signals for 2 outputs
% x0 = [0.1; 0.1; 0.1; 0.1; 0; 0];   % Initial conditions (state + integrator)
% 
% % Simulate using ode45
% [t, x] = ode45(@(t, x) A_aug*x + B_aug*r(:, round(t/0.01)+1), t, x0);
% 
% % Extract outputs
% y = Ccl * x';
% 
% % Plot results
% figure;
% subplot(3, 1, 1);
% plot(t, y(1, :), 'LineWidth', 1.5);
% hold on;
% plot(t, r(1, :), '--', 'LineWidth', 1.5);
% grid on;
% xlabel('Time (s)');
% ylabel('Output y1');
% legend('y1', 'r1');
% title('Output Response for y1');
% 
% subplot(3, 1, 2);
% plot(t, y(2, :), 'LineWidth', 1.5);
% hold on;
% plot(t, r(2, :), '--', 'LineWidth', 1.5);
% grid on;
% xlabel('Time (s)');
% ylabel('Output y2');
% legend('y2', 'r2');
% title('Output Response for y2');
% 
% subplot(3, 1, 3);
% u = -Kx * x(:, 1:nx)' - Ki * x(:, nx+1:end)';
% plot(t, u(1, :), 'LineWidth', 1.5);
% hold on;
% plot(t, u(2, :), '--', 'LineWidth', 1.5);
% grid on;
% xlabel('Time (s)');
% ylabel('Control Inputs');
% legend('u1', 'u2');
% title('Control Inputs');