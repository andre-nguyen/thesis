%% Data load
addpath('bag2matlab');
bag = bagReader('2017-10-24-01-50-06.bag');

%% Trajectory
pose = bag{2, 1};
t = pose.rosbag_recv_time(1:39192);

x = pose.position_x(1:39192);
y = pose.position_y(1:39192);
z = pose.position_z(1:39192);

% Blade step    {1317.40100000000} 2278
% End blade 1   {1352.86500000000} 4051
% Blade 2       {1365.57000000000} 4637
% end blade 2   {1391.57700000000} 5937
% blade 3       {1401.24000000000} 6369
% end blade 3   {1435.82500000000} 8100

t = t - t(1);

%% plot 2d
close all 
figure
subplot(3,1,1);
plot(t, x);
grid on;
xlabel('Temps (s)');
ylabel('x (m)');
title('Contrôle de la distance par rapport à la tour');
subplot(3,1,2);
plot(t, y);
grid on;
xlabel('Temps (s)');
ylabel('y (m)');
title('Centrage de de l''UAV devant la tour');

subplot(3,1,3);
plot(t, z);
grid on;
xlabel('Temps (s)');
ylabel('z (m)');
title('Altitude');


%% Plot
close all;
figure;
plot3(x, y, z);
grid on;
axis equal;