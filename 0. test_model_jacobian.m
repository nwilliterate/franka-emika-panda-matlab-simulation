% Copyright (C) 2021 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Des, 23, 2021
% 
% -------------------------------------------------
% Jacobian Test
% Franka Emika Robot
% Compare the Cartesian derivatives
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
% data load
clc; clear;
addpath(genpath('.'));
task_index = 4;

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

% data 1: real_q -> x_dot
sample_size = length(real_q)-1;
for i=1:sample_size
    [p R] = get_pose(real_q(i,:)');
    
%     % XYZ
%     phi = atan2(R(2,1),R(1,1));
%     theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
%     psi = atan2(R(3,2), R(3,3));
%     eul = [phi; theta; psi];

    % ZYZ  
%     z1 = atan2(R(2,3),R(1,3));
%     y = acos(R(3,3));             % == atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3))
%     z2 = atan2(R(3,2), -R(3,1)); 
%     eul= [z1 y z2]';
    
%     Z2YZ1
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1)); 
    eul = [z2 y z1]';

    kin_x(i,:) = [p; eul];
end

% data 2: real_x -> x_dot
eul =  [];
for i=1:sample_size
    R = quat2rotm([real_x(i,7) real_x(i,4) real_x(i,5) real_x(i,6)]);
    
%     % XYZ
%     phi = atan2(R(2,1),R(1,1));
%     theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
%     psi = atan2(R(3,2), R(3,3));
%     eul(i,:) = [phi; theta; psi];

    % ZYZ  
%     z1 = atan2(R(2,3),R(1,3));
%     y = acos(R(3,3));             % == atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3))
%     z2 = atan2(R(3,2), -R(3,1)); 
%     eul(i,:) = [z1 y z2]';
    
    % Z2YZ1
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1)); 
    eul(i,:) = [z2 y z1]';
end
real_x = [real_x(1:sample_size,1:3), eul];
data1 = (real_x(2:sample_size,:) - real_x(1:sample_size-1,:))/0.001;
data2 = (kin_x(2:sample_size, :) - kin_x(1:sample_size-1, :))/0.001;

% moving average filter
num= 3;
B = 1/num*ones(num,1);
data1 = filter(B,1,data1);
data2 = filter(B,1,data2);

% data3 : jacobians(real_q)*real_dq
for i=1:sample_size-1
%     jacobians = get_JacobianZYX(real_q(i,:));
%     jacobians = get_JacobianZYZ(real_q(i,:));
    jacobians = get_JacobianZ2YZ1(real_q(i,:));
    data3(i,:) = jacobians*[real_dq(i,:)'];
end


t = (1:sample_size-1)*0.001;
% Plotting
figure(1)
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
ylabel_name = ["dx_1", "dx_2", "dx_3", "dx_4", "dx_5", "dx_6"];
for i=1:6
    ax = nexttile;
    hold off
    plot(t, real_jac_dq(2:sample_size,i),'-k','LineWidth',1.5')
    hold on
    plot(t, data1(1:sample_size-1,i),'--c','LineWidth',1)
    hold on
    plot(t, data2(1:sample_size-1,i),':r','LineWidth',1)
    plot(t, data3(1:sample_size-1,i),':b','LineWidth',1)
    xlim([t(1) t(sample_size-1)]);
    if i == 4
        ylim([-pi/4 pi/4])
    end
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    grid on
end
legend('real', 'data1','data2','data3')
lgd = legend;
lgd.Layout.Tile = 9;
lgd.FontSize = 11;
fig_name = "fig\Jac_test_"+test_name+".png";
saveas(gcf,fig_name);
