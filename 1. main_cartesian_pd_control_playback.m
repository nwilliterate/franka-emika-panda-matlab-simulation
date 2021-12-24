% Copyright (C) 2021 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Des, 23, 2021
% 
% -------------------------------------------------
% Cartesian PD Controler
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

% reference joint catersian(euler)
test_name = "catesianPD_playback";
folder_name = "real_data\";
ref_q = table2array(readtable(folder_name+"[210618]joint_postion.csv"));
des_x = table2array(readtable(folder_name+"[210618]cartesian_quat.csv"));

% convent from quat to euler
sample_size = length(des_x);
for i=1:sample_size
    R = quat2rotm([des_x(i,7) des_x(i,4) des_x(i,5) des_x(i,6)]);
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1)); 
    eul(i,:) = [z2 y z1]';
end
des_x = [des_x(1:sample_size,1:3), eul];


% Simulation setup
sim_period = 0.001;
sim_time = sample_size*sim_period;
t = 0.001:sim_period:sim_time;
sample_size = size(t, 2);

% % Cartesian PD Gain 
% Kp=[10 10 10 0.1 0.1 0.1]'*100;
% Kd=sqrt(Kp)*0.5;
% Cartesian PD Gain 
Kp=[20 10 10 1 1 1]'*100;
Kd=sqrt(Kp)*0.75;

% Initialization
x = [ref_q(1, :)'; zeros(7,1)];

% control loop
for i=1:sample_size
    q = x(1:7,i);
    qd = x(8:14,i);
    
    % Franka Kinematics
    % joint space -> task space(euler)
    [p, R]=get_pose(q);
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1)); 
    eul = [z2 y z1]';
    car_pos(i,:) = [p; eul];
    
    % Cartesian Error
    e = des_x(i,:) - car_pos(i,:);

    % Model
    M = get_MassMatrix(q);
    J = get_JacobianZ2YZ1(q);
    
    % model feedback linearization 
    % Computing time too long
    % c = get_CoriolisVector(q, qd); 
    % G = get_GravityVector(q);
    % F = get_FrictionTorque(qd);
    
    % input
    u = M*J'*(Kp.*(e') + Kd.*(J*(-qd)));                    % use simple_rk and simple_plant
    % u = M*pinv(J)*(Kp.*(e) + Kd.*(-J*qd)) + c + G + F;    % use rk and plant
    
    if(i ~= sample_size)
        % rk
        x(:,i+1) = simple_rk(x(:,i), u, sim_period);
        % x(:,i+1) = rk(x(:,i), u, sim_period);
        
        % int
        % x(:, i+1) = x(:, i) + 0.001*(simple_plant(x(:,i), u));
        % x(:, i+1) = x(:, i) + 0.001*(plant(x(:,i), u));
    end
end


% plotting 
% figure 1 : Time 2D Plot
figure(1)
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
ylabel_name = ["p_x(m)", "p_y(m)", "p_z(m)", "r_z", "r_y", "r_z"];
for i=1:6
    ax = nexttile;
    hold off
    plot(t, des_x(:,i),'-r','LineWidth',1.5')
    hold on
    plot(t, car_pos(:,i),'--k','LineWidth',1.')
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    ax = gca;
    r = 0.5;
    xlim([t(1) t(sample_size)]);
    ylim([ax.YLim(1)-r ax.YLim(2)+r])
    grid on
end
legend('real','cal')
lgd = legend;
lgd.Layout.Tile = 9;
lgd.FontSize = 11;
fig_name = "fig\control_"+test_name+".png";
saveas(gcf,fig_name);