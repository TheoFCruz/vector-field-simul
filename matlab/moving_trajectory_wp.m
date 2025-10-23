function [u, V] = moving_trajectory_wp(x, y, R, vel, t)
    dx = x - vel*t;
    dy = y;

    alpha = dx^2 + y^2 - R^2; % Trajectory defintion

    grad = [2*dx, 2*dy];

    % Vector tangent to trajectory
    normg = norm(grad); 
    if normg > 1e-6
        wedge = [-grad(2), grad(1)]/normg; % normalizado
    else
        wedge = [0, 0];
    end

    % P term for time varying
    M = [grad; -grad(2), grad(1)];
    a = [-2*(x-vel*t)*vel; 0];
    P = -(M\a)';

    G = 0.3;
    v_tan = 0.5;
    u = -G*alpha*grad + v_tan*wedge + P; 

    V = abs(alpha);
end