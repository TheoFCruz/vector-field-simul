clear
clc

dt = 0.05; % Simulation step
T = 20; % Simulation span
N = T/dt; % Number of ticks

q = [0.8, -0.5]; % Initial position of the particle
trajectory = zeros(N, 2); % Initialize array to store particle trajectory
trajectory(1, :) = q; % Set initial position in trajectory

for n = 1:N
    velocity = circle_trajectory(q(1), q(2), 2); % Constant velocity in the x-direction
    velocity = max(-1, min(velocity, 1));
    q = q + velocity * dt; % Update position
    trajectory(n + 1, :) = q; % Store new position in trajectory
end

%% Plotting
% Plot the trajectory of the particle
hold on
plot(trajectory(:, 1), trajectory(:, 2), 'LineWidth', 2);

% Set tick font size
ax = gca;
ax.FontSize = 14;

% Set axis
margin = 0.5;
xlims = [min(trajectory(:,1))-margin, max(trajectory(:,1))+margin];
ylims = [min(trajectory(:,2))-margin, max(trajectory(:,2))+margin];
axis([xlims, ylims]);
axis equal;

grid on;
