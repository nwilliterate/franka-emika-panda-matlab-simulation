% Copyright (C) 2021 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Des, 23, 2021
% 
% -------------------------------------------------
% Cartesian Impedance Controller
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

% reference joint catersian(euler)
test_name = "catesian_imp";
folder_name = "real_data\";

% Simulation setup
sim_time = 5;
sim_period = 0.001;
t = 0:sim_period:sim_time;
sample_size = size(t, 2);

% Create a virtual reference trajectory
init_q = [-0.0001   -0.7855    0.0001   -2.3566   -0.0003    3.1419    0.7852]';
[init_p, init_R]=get_pose(init_q);
z1 = atan2(init_R(2,3),init_R(1,3));
y = atan2(sqrt(init_R(1,3)^2 + init_R(2,3)^2), init_R(3,3));
z2 = atan2(init_R(3,2), - init_R(3,1)); 
init_eul= [z2 y z1]';
ref_x = zeros(sample_size, 6);
ref_xd = zeros(sample_size, 6);
ref_xdd = zeros(sample_size, 6);
ref_x(:, 1:3) = [init_p(1)*ones(sample_size,1)'; init_p(2) + 0.2*sin(t); init_p(3) + 0.2*(cos(t) - 1)]';
ref_x(:, 4:6) = init_eul'.* ones(sample_size, 3);
ref_xd(:, 1:3) = [zeros(sample_size,1)'; 0.2*cos(t); -0.2*sin(t)]';
ref_xdd(:, 1:3) = [zeros(sample_size,1)'; -0.2*sin(t); -0.2*cos(t)]';

% ext_force
f_ext = zeros(sample_size, 6);
f_ext(1001:2000, 1) = sin((0.001:0.001:1)*2*pi)*10;

% Impedance Gain
Md = diag([1 1 1 1 1 1]'); 
Kd = diag([20 25 25 200 200 200]'*50); 
Bd = sqrt(Kd)*0.75;

% Initialization
x = [init_q; zeros(7,1)];

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
    
    % Model
    M = get_MassMatrix(q);
    J = get_JacobianZ2YZ1(q);
    J_dot = get_Jacobian_dot(q, qd);
    
    % Cartesian Error
    e = (ref_x(i,:) - car_pos(i,:))';
    e_dot = ref_xd(i,:)' - J*qd;
    
    % model feedback linearization 
    % Computing time too long
    % c = get_CoriolisVector(q, qd); 
    % G = get_GravityVector(q);
    % F = get_FrictionTorque(qd);
    
    % input
    u = M*pinv(J)*(ref_xdd(i,:)+inv(Md)*Bd*(e_dot)+inv(Md)*Kd*e+inv(Md)*f_ext(i,:)'-J_dot*qd)-J'*f_ext(i,:)';   % use simple_rk and simple_plant
    % u = M*pinv(J)*(ref_xdd(i,:)+inv(Md)*Bd*(e_dot)+inv(Md)*Kd*e+inv(Md)*f_ext(i,:)'-J_dot*qd)-J'*f_ext(i,:)';    % use rk and plant
    
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
    plot(t, ref_x(:,i),'-r','LineWidth',1.5')
    hold on
    plot(t, car_pos(:,i),'--k','LineWidth',1.')
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    ax = gca;
    r = 0.025;
    xlim([t(1) t(sample_size)]);
    ylim([ax.YLim(1)-r ax.YLim(2)+r])
    grid on
end
legend('ref','cur')
lgd = legend;
lgd.Layout.Tile =9;
lgd.FontSize = 11;
fig_name = "fig\control_"+test_name+".png";
saveas(gcf,fig_name);

% plotting 
% figure 2: 3D Cartesian Pose Plot
figure(2)
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
nexttile
hold off
plot3(ref_x(:,1), ref_x(:,2),ref_x(:,3),'-r','LineWidth',1.5')
hold on
plot3(car_pos(:,1), car_pos(:,2),car_pos(:,3),'--k','LineWidth',1')
scale = 0.00075;
legend('ref x', 'cur x','Location','northeast')
lgd = legend;
lgd.FontSize = 11;
axis([0.275 0.375 -0.5 0.5 0.1 1]);
xlabel('p_x(m)', 'FontSize', 10)
ylabel("p_y(m)", 'FontSize', 10);
zlabel('p_z(m)', 'FontSize', 10)
grid on;
