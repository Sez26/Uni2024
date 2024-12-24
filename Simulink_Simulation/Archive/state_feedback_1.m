% state_feedback attempt 2
Initialise_var_0
t = 0:0.01:2;
u = zeros(length(t), 2);
x0 = [0.01 0 0 0];

sys = ss(A,B,C,D);

[y,t,x] = lsim(sys,u,t,x0);
plot(t,y)
title('Open-Loop Response to Non-Zero Initial Condition')
xlabel('Time (sec)')
ylabel('Ball Position (m)')

% Desired settling time (Ts) in seconds
Ts = 2; % Specify your desired settling time

% Calculate desired pole locations
zeta = 0.7; % Damping ratio
omega_n = 4 / (zeta * Ts); % Natural frequency
% Assume four complex-conjugate poles for demonstration
desired_poles = [-zeta * omega_n + omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -zeta * omega_n - omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -2 * zeta * omega_n, -3 * zeta * omega_n];


% Check controllability
ctrb_matrix = ctrb(A, B);
if rank(ctrb_matrix) < size(A, 1)
    error('The system is not controllable and cannot be stabilized.');
end

K = place(A, B, desired_poles);

sys_cl = ss(A-B*K,B,C,0);

lsim(sys_cl,u,t,x0);
xlabel('Time (sec)')
ylabel('Ball Position (m)')

Nbar = rscale(sys,K);