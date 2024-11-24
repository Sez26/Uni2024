% ref fitting matlab
origin_sq = [0.5, 0.0];
sq_sl = 0.086;
% get square way points
% first row x
% secod row y
wp_sq = [origin_sq(1)-(sq_sl/2) origin_sq(1)+(sq_sl/2) origin_sq(1)+(sq_sl/2) origin_sq(1)-(sq_sl/2) origin_sq(1)-(sq_sl/2); ...
    origin_sq(2)-(sq_sl/2) origin_sq(2)-(sq_sl/2) origin_sq(2)+(sq_sl/2) origin_sq(2)+(sq_sl/2) origin_sq(2)-(sq_sl/2)];

[q,qd,qdd,tvec,pp] = trapveltraj(wp_sq,1000);

subplot(2,1,1)
plot(tvec, q)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(2,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')

[q, qd, qdd, pp] = quinticpolytraj(wp_sq, tpts, tvec);
subplot(2,1,1)
plot(tvec, q)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(2,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')

figure
plot(q(1,:),q(2,:),'-b|',wp_sq(1,:),wp_sq(2,:),'or')

% get theta values



function [theta_1, theta_2] = get_thetas(tar_x, tar_y, L_1, L_2)
    % Using cosine rule to find internal angles alpha_1, alpha_2
    phi = atan2(tar_y, tar_x);
    c_sq = tar_x^2 + tar_y^2;
    alpha_1 = acos((c_sq + L_1^2 - L_2^2) / (2 * L_1 * sqrt(c_sq)));
    alpha_2 = acos((L_1^2 + L_2^2 - c_sq) / (2 * L_1 * L_2));
    
    % Convert to global linkage angles theta_1, theta_2
    theta_1 = phi + alpha_1;
    theta_2 = pi + phi + alpha_1 + alpha_2;
    
    % Convert to degrees
    theta_1 = rad2deg(theta_1);
    theta_2 = rad2deg(theta_2);
end