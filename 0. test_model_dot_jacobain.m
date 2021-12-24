% Copyright (C) 2021 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Des, 23, 2021
%
% -------------------------------------------------
% Jacobian derivatives Test
% Franka Emika Robot
% Compare the two dot x
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%% Data load
clc; clear;
addpath(genpath('.'));
task_index = 1;

% Data Load
folder_name = "real_data\";

if task_index == 1
    test_name = "simple_test";
    time_line = "[210618]";
elseif task_index == 2
    test_name = "joint1~7 movement";
    time_line = "[20210709-1411]franka_data_";
elseif task_index == 3
    test_name = "joint7 movement";
    time_line = "[20210709-1436]franka_data_";
elseif task_index == 4
    test_name = "joint5 movement";
    time_line = "[20210709-1437]franka_data_";
end

real_q = table2array(readtable(folder_name+time_line+"joint_postion.csv"));
real_dq = table2array(readtable(folder_name+time_line+"joint_velocity.csv"));
real_x = table2array(readtable(folder_name+time_line+"cartesian_quat.csv"));
real_jac_dq = table2array(readtable(folder_name+time_line+"jac_vel_product.csv"));

% Convert from quaternion to Euler ZYZ.
eul =  [];
sample_size = length(real_q)-2;

for i=1:sample_size
    R = quat2rotm([real_x(i,7) real_x(i,4) real_x(i,5) real_x(i,6)]);
    % Z2YZ1
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1));
    eul(i,:) = [z2 y z1]';
end

% dx, ddx, ddq
real_x_eul = [real_x(1:sample_size,1:3), eul];
dot_x = (real_x_eul(2:sample_size,:) - real_x_eul(1:sample_size-1,:))/0.001;
ddot_x = (dot_x(2:sample_size-1,:) - dot_x(1:sample_size-2,:))/0.001;
ddq =  (real_dq(2:sample_size,:) - real_dq(1:sample_size-1,:))/0.001;

% Set the same sample size
sample_size = length(ddot_x);
q = real_q(1:sample_size, :);
dq = real_dq(1:sample_size, :);
ddq = ddq(1:sample_size, :);
x = real_x_eul(1:sample_size, :);
dx =  dot_x(1:sample_size, :);
ddx = ddot_x(1:sample_size, :);

% Apply the moving average filter
num= 1000;
B = 1/num*ones(num,1);
q = filter(B,1,q);
dq = filter(B,1,dq);
ddq = filter(B,1,ddq);
x = filter(B,1,x);
dx = filter(B,1,dx);
ddx = filter(B,1,ddx);

% J, dJ
sample_size = length(q);
for i=1:sample_size
    J = get_JacobianZ2YZ1(q(i,:));
    J_dot = get_Jacobian_dot(q(i,:), dq(i,:));
    com_ddx(:,i) = J*ddq(i,:)' + J_dot*dq(i,:)';
end
com_ddx = com_ddx';


t = (1:sample_size)*0.001;
% Plotting
figure(1)
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
ylabel_name = ["ddx_1", "ddx_2", "ddx_3", "ddx_4", "ddx_5", "ddx_6"];
for i=1:6
    ax = nexttile;
    hold off
    plot(t, ddx(:,i),':k','LineWidth',1.5')
    hold on
    plot(t, com_ddx(:,i),'--r','LineWidth',1.5)
    xlim([t(1) t(sample_size)]);
    if i == 4
        ylim([-pi/2 pi/2])
    end
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    grid on
end
legend('real', 'com')
lgd = legend;
lgd.Layout.Tile = 9;
lgd.Layout.Tile = 9;
lgd.FontSize = 11;
fig_name = "fig\Jac_dot_test_"+test_name+".png";
saveas(gcf,fig_name,'epsc');