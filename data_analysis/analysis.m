%% Data load
addpath('bag2matlab');
bag = bagReader('2017-10-24-01-50-06.bag');

%% Trajectory
pose = bag{2, 1};
t = pose.rosbag_recv_time(1:39192);
tower_ascent_end_time = t(39192);

x = pose.position_x(1:39192);
y = pose.position_y(1:39192);
z = pose.position_z(1:39192);

% Blade step    {1317.40100000000} 2278
% End blade 1   {1352.86500000000} 4051
% Blade 2       {1365.57000000000} 4637
% end blade 2   {1391.57700000000} 5937
% blade 3       {1401.24000000000} 6369
% end blade 3   {1435.82500000000} 8100



cmd = bag{1,1};
idx = find(cmd.rosbag_recv_time<tower_ascent_end_time, 1, 'last');
t_cmd = cmd.rosbag_recv_time(1:idx);
vx = cmd.linear_x(1:idx);
vy = cmd.linear_y(1:idx);
vz = cmd.linear_z(1:idx);

t_cmd = t_cmd - t_cmd(1);
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
xlim([0 40]);

subplot(3,1,2);
plot(t, y);
grid on;
xlabel('Temps (s)');
ylabel('y (m)');
title('Centrage de de l''UAV devant la tour');
xlim([0 40]);

subplot(3,1,3);
plot(t, z);
grid on;
xlabel('Temps (s)');
ylabel('z (m)');
title('Altitude');
xlim([0 40]);

% with velocities
figure
subplot(3,2,1);
plot(t, x);
grid on;
xlabel('Temps (s)');
ylabel('x (m)');
title('Contrôle de la distance par rapport à la tour');
xlim([0 40]);

subplot(3,2,3);
plot(t, y);
grid on;
xlabel('Temps (s)');
ylabel('y (m)');
title('Centrage de de l''UAV devant la tour');
xlim([0 40]);

subplot(3,2,5);
plot(t, z);
grid on;
xlabel('Temps (s)');
ylabel('z (m)');
title('Altitude');
xlim([0 40]);

subplot(3,2,2);
plot(t_cmd, vx);
grid on;
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
title('Vitesse en x');
xlim([0 40]);

subplot(3,2,4);
plot(t_cmd, vy);
grid on;
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
title('Vitesse en y');
xlim([0 40]);

subplot(3,2,6);
plot(t_cmd, vz);
grid on;
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
title('Vitesse en z');
xlim([0 40]);

%% Plot
close all;
colors = parula;
color_idx = [1 15 30 40 45 64];
t = pose.rosbag_recv_time(:);

e_idx = find(t < 1317.40100000000, 1, 'last');

x = pose.position_x(1:e_idx);
y = pose.position_y(1:e_idx);
z = pose.position_z(1:e_idx);

% Blade step    {1317.40100000000} 2278
% End blade 1   {1352.86500000000} 4051
% Blade 2       {1365.57000000000} 4637
% end blade 2   {1391.57700000000} 5937
% blade 3       {1401.24000000000} 6369
% end blade 3   {1435.82500000000} 8100

figure;
plot3(x, y, z, 'Color', colors(color_idx(1), :));
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
hold on;

% First blade
s_idx = e_idx + 1;
e_idx = find(t < 1352.86500000000, 1, 'last');
x = pose.position_x(s_idx:e_idx);
y = pose.position_y(s_idx:e_idx);
z = pose.position_z(s_idx:e_idx);
plot3(x, y, z, 'Color', colors(color_idx(2), :));

% Return to rotor
s_idx = e_idx + 1;
e_idx = find(t < 1365.5700000000, 1, 'last');
x = pose.position_x(s_idx:e_idx);
y = pose.position_y(s_idx:e_idx);
z = pose.position_z(s_idx:e_idx);
plot3(x, y, z, '--', 'Color', colors(color_idx(5), :));

% Second blade
s_idx = e_idx + 1;
e_idx = find(t < 1391.57700000000, 1, 'last');
x = pose.position_x(s_idx:e_idx);
y = pose.position_y(s_idx:e_idx);
z = pose.position_z(s_idx:e_idx);
plot3(x, y, z, 'Color', colors(color_idx(3), :));

% return to rotor
s_idx = e_idx + 1;
e_idx = find(t < 1401.24000000000, 1, 'last');
x = pose.position_x(s_idx:e_idx);
y = pose.position_y(s_idx:e_idx);
z = pose.position_z(s_idx:e_idx);
plot3(x, y, z, '--', 'Color', colors(color_idx(5), :));

% 3rd blade
s_idx = e_idx + 1;
e_idx = find(t < 1435.82500000000, 1, 'last');
x = pose.position_x(s_idx:e_idx);
y = pose.position_y(s_idx:e_idx);
z = pose.position_z(s_idx:e_idx);
plot3(x, y, z, 'Color', colors(color_idx(4), :));

% return to rotor
s_idx = e_idx + 1;
e_idx = find(t > 1435.82500000000, 1, 'last');
x = pose.position_x(s_idx:e_idx);
y = pose.position_y(s_idx:e_idx);
z = pose.position_z(s_idx:e_idx);
plot3(x, y, z, '--', 'Color', colors(color_idx(5), :));

legend( 'Montée de la tour', ...
        'Pale 1', ...
        'retour au rotor', ...
        'Pale 2', ...
        'retour au rotor', ...
        'Pale 3', ...
        'retour au rotor');
        
%% Blade plot
start = find(pose.rosbag_recv_time >  1317.40100000000, 1);
finish = find(pose.rosbag_recv_time > 1352.86500000000, 1);

t = pose.rosbag_recv_time(start:finish+1);
finish_time = t(end);

x = pose.position_x(start:finish+1);
y = pose.position_y(start:finish+1);
z = pose.position_z(start:finish+1);

start = find(cmd.rosbag_recv_time >  1317.40100000000, 1);
finish = find(cmd.rosbag_recv_time > 1352.86500000000, 1);
t_cmd = cmd.rosbag_recv_time(start:finish+1);
vx = cmd.linear_x(start:finish+1);
vy = cmd.linear_y(start:finish+1);
vz = cmd.linear_z(start:finish+1);

t_cmd = t_cmd - t_cmd(1);
t = t - t(1);

% with velocities
figure
subplot(3,2,1);
plot(t, x);
grid on;
xlabel('Temps (s)');
ylabel('x (m)');
title('Contrôle de la distance par rapport à la pale 1');

subplot(3,2,3);
plot(t, y);
grid on;
xlabel('Temps (s)');
ylabel('y (m)');
title('Suivit le long de la pale 1');
xlim([0 40]);

subplot(3,2,5);
plot(t, z);
grid on;
xlabel('Temps (s)');
ylabel('z (m)');
title('Centrage de l''UAV en z par rapport à la pale 1');
xlim([0 40]);

subplot(3,2,2);
plot(t_cmd, vx);
grid on;
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
title('Vitesse en x');
xlim([0 40]);

subplot(3,2,4);
plot(t_cmd, vy);
grid on;
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
title('Vitesse en y');
xlim([0 40]);

subplot(3,2,6);
plot(t_cmd, vz);
grid on;
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
title('Vitesse en z');
xlim([0 40]);









