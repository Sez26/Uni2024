%% Statefeedback Control
Initialise_var_0
% from linearised A and B matrices
% tune controller matrix K

% Desired settling time (Ts) in seconds
Ts = 0.5; % Specify your desired settling time

% get desired poles from PID tuning
comp_poles = pole(linsys3);

% cutting poles down to 4 determining poles
pol_idx = [3:6];
desired_poles = comp_poles(pol_idx);

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