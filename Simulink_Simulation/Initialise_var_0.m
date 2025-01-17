%%% YOU HAVE RUN THIS FILE BEFORE RUNNING SIMULINK %%%

addpath ref_source\

%% System parameters
% in SI units
m_a1 = 0.018+0.015;
m_a2 = 0.0215+0.0156;
m_m2 = 0.205;
m_p = 0.0204+0.0058+0.213;
L1 = 0.095;
L2 = 0.095;
g = 9.81;

% calculated values
% combined masses
m1 = m_a1 + m_m2;
m2 = m_a2 + m_p;

% distance to CoM in links (UPDATE THIS EQ)
lc1 = L1/2;
lc2 = L2/2;

% calculate moments of inertia of the arms (uniform bars with point masses on the end)
I1 = (1/3)*m_a1*L1^2 + m_m2*L1^2;
I2 = (1/3)*m_a2*L2^2 + m_p*L2^2;
%% Linearised Plant (about position equlibrium)

% state variable = [theta1, theta2, dtheta1, dtheta2] !!!!!!!!!!!!!

theta_eq = [-pi/4; -pi/4];  % Equilibrium joint angles
dtheta_eq = [0; 0]; % Equilibrium velocity

tau1_eq = lc1*sin(theta_eq(1))*m1*g;
tau2_eq = lc2*sin(theta_eq(2))*m2*g;
tau_eq = [0; 0];    % Equilibrium torques

function [A, B] = linearize_2link(theta_eq, dtheta_eq, m1, m2, lc1, lc2, l1, l2, I1, I2, g)
    % Equilibrium point
    dtheta1 = dtheta_eq(1); dtheta2 = dtheta_eq(2);
    theta1 = theta_eq(1); theta2 = theta_eq(2);
    
    % Mass matrix M(theta)
    M11 = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(theta2)) + I1 + I2;
    M12 = m2*(lc2^2 + l1*lc2*cos(theta2)) + I2;
    M22 = m2*lc2^2 + I2;
    M = [M11, M12; M12, M22];

    % Coriolis/centrifugal terms C(theta, dtheta)
    C11 = -m2*l1*lc2*sin(theta2)*dtheta2;
    C12 = -m2*l1*lc2*sin(theta2)*(dtheta1 + dtheta2);
    C21 = m2*l1*lc2*sin(theta2)*dtheta1;
    C22 = 0;
    C = [C11, C12; C21, C22];

    % Gravity vector G(theta)
    G1 = (m1*lc1 + m2*l1)*g*cos(theta1) + m2*lc2*g*cos(theta1 + theta2);
    G2 = m2*lc2*g*cos(theta1 + theta2);
    G = [G1; G2];

    % Partial derivatives of G(theta)
    dG_dtheta1 = -(m1*lc1 + m2*l1)*g*sin(theta1) - m2*lc2*g*sin(theta1 + theta2);
    dG_dtheta2 = -m2*lc2*g*sin(theta1 + theta2);
    dG = [dG_dtheta1, dG_dtheta2; 0, dG_dtheta2];

    % Compute A and B matrices
    A = zeros(4, 4);
    A(1, 3) = 1; % d(theta1)/dt
    A(2, 4) = 1; % d(theta2)/dt
    A(3:4, 1:2) = -M \ dG; % d^2(theta)/dt^2 w.r.t theta
    A(3:4, 3:4) = -M \ C;  % d^2(theta)/dt^2 w.r.t dtheta

    B = zeros(4, 2);
    B(3:4, :) = M \ eye(2); % d^2(theta)/dt^2 w.r.t tau
end

[A, B] = linearize_2link(theta_eq, dtheta_eq, m1, m2, lc1, lc2, L1, L2, I1, I2, g);

%% Setting C and D matrices
C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; % the perfect state feedback
D = [0 0; 0 0; 0 0; 0 0]; % fucking control coupling

% C = [1 0 0 0; 0 1 0 0]; % the perfect state feedback
% D = [0 0; 0 0]; % fucking control coupling