% Copyright (C) 2021 Electronics and Telecommunications Research Institute(ETRI). All rights reserved.
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:        Ang, 10, 2021
% 
% -------------------------------------------------
% Runge-Kutta Method
% -------------------------------------------------
% Equation)
%       dx = Ax+Bu
% Input)
%       x   : state x
%       u   : Input u
%       T   : Simulation Time 
% Output)
%       dx  : Differentiation of State x
%
% the following code has been tested on Matlab 2020b
function dx = simple_rk(x, u, T)
k1 = simple_plant(x, u)*T;
k2 = simple_plant(x+k1*0.5, u)*T;
k3 = simple_plant(x+k2*0.5, u)*T;
k4 = simple_plant(x+k3, u)*T;
dx = x+((k1+k4)/6+(k2+k3)/3);
end