% ref fitting matlab
origin_sq = [0.5, 0.0];
num_pts = 1000;
sq_sl = 0.086;
% get square way points
% first row x
% secod row y
wp_sq = [origin_sq(1)-(sq_sl/2) origin_sq(1)+(sq_sl/2) origin_sq(1)+(sq_sl/2) origin_sq(1)-(sq_sl/2) origin_sq(1)-(sq_sl/2); ...
    origin_sq(2)-(sq_sl/2) origin_sq(2)-(sq_sl/2) origin_sq(2)+(sq_sl/2) origin_sq(2)+(sq_sl/2) origin_sq(2)-(sq_sl/2)];

[q_trap,qd_trap,qdd_trap,tvec_trap,pp_trap] = trapveltraj(wp_sq,200);

figure
subplot(3,1,1)
plot(tvec_trap, q_trap)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(3,1,2)
plot(tvec_trap, qd_trap)
xlabel('t')
ylabel('Velocities')
legend('X','Y')
subplot(3,1,3)
plot(tvec_trap, qdd_trap)
xlabel('t')
ylabel('Accelerations')
legend('X','Y')

tpts = 0:0.5:2;

tvec = 0:0.01:2;

figure

[q_poly, qd_poly, qdd_poly, pp_poly] = quinticpolytraj(wp_sq, tpts, tvec);
subplot(3,1,1)
plot(tvec, q_poly)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(3,1,2)
plot(tvec, qd_poly)
xlabel('t')
ylabel('Velocities')
legend('X','Y')
subplot(3,1,3)
plot(tvec, qdd_poly)
xlabel('t')
ylabel('Accelerations')
legend('X','Y')


figure
% plot(q_trap(1,:),q_trap(2,:),'-r*')
hold on
plot(q_trap(1,:),q_trap(2,:),'-bo',wp_sq(1,:),wp_sq(2,:),'or')
hold off

% get theta values

L_1 = 0.095;
L_2 = 0.095;

function [theta_1, theta_2] = get_thetas(tar_x, tar_y, L_1, L_2)
    % Using cosine rule to find internal angles alpha_1, alpha_2
    phi = atan2(tar_y, tar_x);
    c_sq = tar_x.^2 + tar_y.^2;
    alpha_1 = acos((c_sq + L_1^2 - L_2^2) / (2 * L_1 * sqrt(c_sq)));
    alpha_2 = acos((L_1^2 + L_2^2 - c_sq) / (2 * L_1 * L_2));
    
    % Convert to global linkage angles theta_1, theta_2
    theta_1 = phi + alpha_1;
    theta_2 = pi + phi + alpha_1 + alpha_2;
    
    % Convert to degrees
    theta_1 = rad2deg(theta_1);
    theta_2 = rad2deg(theta_2);
end

[th1,th2] = get_thetas(q_poly(1,:), q_poly(2,:), L_1, L_2);

figure
plot(tvec,th1,tvec,th2)