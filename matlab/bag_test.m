clear;
clc;

% Creates bag object and gets the messages
bag = ros2bagreader('bags/moving_bag1/');
msgs = readMessages(bag);

% Vectors for the x and y positions
n = numel(msgs);
x = zeros(n,1);
y = zeros(n,1);

for i = 1:n
    x(i) = msgs{i}.pose.pose.position.x;
    y(i) = msgs{i}.pose.pose.position.y;
end

% Plots
figure; hold on; grid on; axis equal;
plot(x,y,'b-','LineWidth',1.5);
xlabel('X [m]'); ylabel('Y [m]');
title('Trajetória do robô (rosbag2 /odom)');