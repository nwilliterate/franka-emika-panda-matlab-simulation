% Copyright (C) 2021 Electronics and Telecommunications Research Institute(ETRI). All rights reserved.
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:        Ang, 10, 2021
% 
% -------------------------------------------------
% Simple PD Controler
% Franka Emika Robot
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

% reference trajectory - real franka data
folder_name = "real_data\";
ref_q = table2array(readtable(folder_name+"[210618]joint_postion_data.csv"));
sample_size = length(ref_q);

% simulation
sim_period = 0.001;
sim_time = sample_size*sim_period;
t = 0:sim_period:sim_time;
sample_size = size(t, 2);

% PD gain 
Kp=[5 10 10 5 1 1 1]'*10;
Kd=sqrt(Kp)*0.5;

x = [ref_q(1, :)'; zeros(7,1)];

% control loop
for i=1:sample_size-1
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
    % u = (Kp.*(e) + Kd.*(ed)) + c + G + F;   % use rk and plant
    
    
    if(i ~= sample_size)
        % rk
        x(:,i+1) = simple_rk(x(:,i), u, sim_period);
        % x(:,i+1) = rk(x(:,i), u, sim_period);
        
        % int
        % x(:, i+1) = x(:, i) + 0.001*(simple_plant(x(:,i), u));
        % x(:, i+1) = x(:, i) + 0.001*(plant(x(:,i), u));
    end
end

% Plotting
figure(1)
set(gcf,'color','w');
for i=1:7
    subplot(7,1,i)
    hold off
    plot(ref_q(:,i),'-k','LineWidth',1.5')
    hold on;
    plot(x(i,:),'--r','LineWidth',1.5')
    grid on;
end