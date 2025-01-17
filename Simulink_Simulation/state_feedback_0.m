%% Statefeedback Control
Initialise_var_0
% from linearised A and B matrices
% tune controller matrix K

% Desired settling time (Ts) in seconds
Ts = 0.0625; % Specify your desired settling time

% Calculate desired pole locations
zeta = 0.2; % Damping ratio
omega_n = 4 / (zeta * Ts); % Natural frequency
% Assume four complex-conjugate poles for demonstration
desired_poles = [-zeta * omega_n + omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -zeta * omega_n - omega_n * sqrt(1 - zeta^2) * 1i, ...
                 -5 * zeta * omega_n, -6 * zeta * omega_n];

% Check controllability
ctrb_matrix = ctrb(A, B);
if rank(ctrb_matrix) < size(A, 1)
    error('The system is not controllable and cannot be stabilized.');
end

% Compute state-feedback gain K using pole placement
% Since B has multiple columns, use the 'place' function for MIMO
K = place(A, B, desired_poles);

% Display the results
disp('State-feedback gain K:');
disp(K);

% Verify the closed-loop system eigenvalues
A_cl = A - B * K; % Closed-loop system matrix
disp('Closed-loop eigenvalues:');
disp(eig(A_cl));

E = [1 0 0 0; 0 1 0 0];
K_r1 = inv(A-B*K);
K_r = -inv(E*inv(A-B*K)*B);