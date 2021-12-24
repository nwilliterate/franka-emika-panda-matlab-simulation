% Copyright (C) 2021 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Des, 23, 2021
% 
% -------------------------------------------------
% Kinematics Test
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));


% data load
test_name = "real_data\kin_test_real_data_";
real_x = table2array(readtable(test_name+"catesian.csv"));  % [px py pz rx ry rz rw]
real_q = table2array(readtable(test_name+"joint_angle.csv")); % [q1 q2 q3 q4 q5 q6 q7]

% kinamtics test
sample_size = length(real_q);
for i=1:sample_size
    [p, R] = get_pose(real_q(i,:)'); 
    
    x(i,1:3) = p;
    quat = -rotm2quat(R); %[w x y z]
    x(i,4:7) = [quat(2:4) quat(1)]; % [x y z w]
end
t = (1:sample_size)*0.001;

% plotting 
% figure 1 : Time 2D Plot
figure(1)
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
ylabel_name = ["p_x", "p_y(m)", "p_z(m)", "r_x", "r_y", "r_z", "r_w"];
for i=1:7
    ax = nexttile;
    hold off
    plot(t, real_x(:,i),'-r','LineWidth',1.5')
    hold on
    plot(t, x(:,i),'--k','LineWidth',1.')
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    ax = gca;
    r = 0.5;
    xlim([t(1) t(sample_size)]);
%     ylim([ax.YLim(1)-r ax.YLim(2)+r])
    grid on
end
legend('real','cal')
lgd = legend;
lgd.Layout.Tile = 8;
saveas(gcf,'fig\kin_test1.eps','epsc');

% figure 2 : 3D Plot 
figure(2)
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
hold off
plot3(real_x(:,1), real_x(:,2), real_x(:,3),'-r','LineWidth',1.5')
hold on
plot3(x(:,1), x(:,2), x(:,3),'--k','LineWidth',1.')
grid on;
ax = gca;
r = 0.075;
axis([ax.XLim(1)-r ax.XLim(2)+r ax.YLim(1)-r ax.YLim(2)+r ax.ZLim(1)-r ax.ZLim(2)+r])
xlabel('P_x(m)','FontSize', 12);
ylabel('P_y(m)','FontSize', 12);
zlabel('P_z(m)','FontSize', 12);
saveas(gcf,'fig\kin_test2.eps','epsc');
