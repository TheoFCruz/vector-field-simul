clear
clc

dt = 0.05; % Simulation step
T = 80; % Simulation span
N = T/dt; % Number of ticks

q = [0, 0]; % Initial position of the particle
trajectory_err = zeros(N, 2); % Initialize array to store particle trajectory
trajectory_err(1, :) = q; % Set initial position in trajectory
potential_err = zeros(N,1);

% Trajectory without correction term
for n = 1:N
    [velocity, V] = moving_trajectory_err(q(1), q(2), -0.8, 0.5, 2, 0.1, n*dt); % Constant velocity in the x-direction
    velocity = max(-0.46, min(velocity, 0.46));
    q = q + velocity * dt; % Update position
    trajectory_err(n + 1, :) = q; % Store new position in trajectory
    potential_err(n) = V;
end

q = [0, 0];
trajectory_wp = zeros(N,2);
potential_wp = zeros(N,1);

% Trajectory with correction term
for n = 1:N
    [velocity, V] = moving_trajectory_wp(q(1), q(2), -0.8, 0.5, 2, 0.1, n*dt); % Constant velocity in the x-direction
    velocity = max(-0.46, min(velocity, 0.46));
    q = q + velocity * dt; % Update position
    trajectory_wp(n + 1, :) = q; % Store new position in trajectory
    potential_wp(n) = V;
end

%% Plotting
% Plot the trajectories of the particle
hold on
grid on
plot(trajectory_err(: , 1), trajectory_err(: , 2), ...
    '--b', 'LineWidth', 1);

plot(trajectory_wp(: , 1), trajectory_wp(: , 2), ...
    '-b', 'LineWidth', 2);
plot(trajectory_wp(N, 1), trajectory_wp(N, 2), ...
    'ob', 'MarkerSize', 16, 'LineWidth', 2);

% Set tick font size
ax = gca;
ax.FontSize = 14;

% Set axis
margin = 0.5;
xlims = [min(trajectory_wp(:,1))-margin, max(trajectory_wp(:,1))+margin];
ylims = [min(trajectory_wp(:,2))-margin, max(trajectory_wp(:,2))+margin];
axis([xlims, ylims]);
axis equal;

% Potential curve
figure(2);
hold on;
plot(1:N, potential_err, '--b', 'LineWidth', 1);
plot(1:N, potential_wp, '-b', 'LineWidth', 2);
ax = gca;
ax.FontSize = 14;

grid on;
