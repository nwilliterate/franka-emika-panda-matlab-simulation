% Copyright (C) 2021 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Des, 23, 2021
% 
% -------------------------------------------------
% Simple PD Controler playback
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

% reference trajectory - real franka data
test_name = "simplePD_playback";
folder_name = "real_data\";
ref_q = table2array(readtable(folder_name+"[210618]joint_postion.csv"));
sample_size = length(ref_q);

% simulation
sim_period = 0.001;
sim_time = sample_size*sim_period;
t = 0.001:sim_period:sim_time;
sample_size = size(t, 2);

% PD gain 
Kp=[5 10 10 5 1 1 1]'*10;
Kd=sqrt(Kp)*0.5;

x = [ref_q(1, :)'; zeros(7,1)];

% control loop
for i=1:sample_size
    q = x(1:7,i);
    qd = x(8:14,i);
    
    % joint error
    e = ref_q(i,:)' - q;
    ed = 0 - qd;
    
    % model feedback linearization 
    % Computing time too long
    % c = get_CoriolisVector(q, qd); 
    % G = get_GravityVector(q);
    % F = get_FrictionTorque(qd);
    
    
    % input
    u = (Kp.*(e) + Kd.*(ed));                 % use simple_rk and simple_plant
    % u = (Kp.*(e) + Kd.*(ed)) + c + G + F;       % use rk and plant
    
    
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
% ylabel_name = ["q_1(rad)", "p_y(m)", "p_z(m)", "r_x", "r_y", "r_z", "r_w"];
for i=1:7
    ax = nexttile;
    hold off
    plot(t, ref_q(:,i),'-r','LineWidth',1.5')
    hold on
    plot(t, x(i,:),'--k','LineWidth',1.')
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("q_"+num2str(i)+"(rad)", 'FontSize', 10);
    ax = gca;
    r = 0.5;
    xlim([t(1) t(sample_size)]);
    ylim([ax.YLim(1)-r ax.YLim(2)+r])
    grid on
end
legend('ref','cur')
lgd = legend;
lgd.Layout.Tile = 9;
lgd.FontSize = 11;
fig_name = "fig\control_"+test_name+".png";
saveas(gcf,fig_name);