% Copyright (C) 2021 Electronics and Telecommunications Research Institute(ETRI). All rights reserved.
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:        Ang, 10, 2021
% 
% -------------------------------------------------
% Franka Emika Forward Kinematics
% -------------------------------------------------
% Input)
%       q   : Joint Postion 
% Output
%       p   : Cartesian Postion  
%       R   : Cartesian Rotation Matrix 
%
% the following code has been tested on Matlab 2020b
function [p, R]=get_pose(q)
q = [q; 0];
n = 8;
d = [0.333 0 0.316 0 0.384 0 0 0.107]';
a = [0 0 0 0.0825 -0.0825 0 0.088 0]';
alpha = [0 -pi/2 pi/2 pi/2 -pi/2 pi/2 pi/2 0 ]';


Ti= cell(n,1); 
for j=1:n
    if(j == 1)
        Ti{j} = Rhx(alpha(j)) * Rt(a(j), 0, 0)  * Rhz(q(j)) * Rt(0,0,d(j));
    else
        Ti{j} = Ti{j-1}*Rhx(alpha(j)) * Rt(a(j), 0, 0)  * Rhz(q(j)) * Rt(0,0,d(j));
   end
end
p = Ti{n}(1:3,4);
R = Ti{n}(1:3,1:3);
end

function [T] = Rhx(theta)
T = [ 1 0 0 0;
    0 cos(theta) -sin(theta) 0; 
    0 sin(theta) cos(theta) 0; 
    0 0 0 1
    ];
end
function [T] = Rhy(theta)
T = [ cos(theta) 0 sin(theta) 0; 
    0 1 0 0;   
    -sin(theta) 0 cos(theta) 0 ; 
    0 0 0 1
    ];
end
function [T] = Rhz(theta)
T = [ cos(theta) -sin(theta) 0 0; 
    sin(theta) cos(theta) 0 0;
    0 0 1 0; 
    0 0 0 1
    ];
end
function [T] = Rt(x, y, z)
T = [ 1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
end