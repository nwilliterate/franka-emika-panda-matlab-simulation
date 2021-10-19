% Copyright (C) 2021 Electronics and Telecommunications Research Institute(ETRI). All rights reserved.
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:        Ang, 10, 2021
% 
% -------------------------------------------------
% Franka Emika Plant Function
% -------------------------------------------------
% Equation)
%       ddq = M(q)ddq + c(q, dq)dq + g(q) + f(dq)
%
% Input)
%       x   : Joint Postion and Velocity
%       u   : Joint Torque
% Output)
%       dxdt  : Joint Velocity and Acceleration
%
% the following code has been tested on Matlab 2020b
function dxdt = plant(x,u)
q = x(1:7);
qd = x(8:14);

M = get_MassMatrix(q);
c = get_CoriolisVector(q, qd);
G = get_GravityVector(q);
F = get_FrictionTorque(qd);

temp_dt = inv(M)*(-c-G-F+u);
dxdt = [x(8);x(9);x(10);x(11);x(12);x(13);x(14);temp_dt(1);temp_dt(2);temp_dt(3);temp_dt(4);temp_dt(5);temp_dt(6);temp_dt(7)];
end