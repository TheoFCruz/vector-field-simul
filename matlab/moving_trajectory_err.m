function [u, V] = moving_trajectory_err(x, y, R, vel, t)
    alpha = (x- vel*t)^2 + y^2 - R^2; % Trajectory defintion

    grad = [2*(x-vel*t), 2*y]; % Alpha gradient

    % Vector tangent to trajectory
    normg = norm(grad); 
    if normg > 1e-6
        wedge = [-grad(2), grad(1)]/normg; % normalizado
    else
        wedge = [0, 0];
    end

    G = 0.1;
    v_tan = 0.3;
    u = -G*alpha*grad + v_tan*wedge; 

    V = abs(alpha);
end