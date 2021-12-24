# Franka Emika Panda Matlab Simulation

This repository is a MATLAB simulation of franka emika panda robot control using Runge-Kutta. The robot manipulator dynamic model (M, C, G, F) uses the functions provided in [1]. The kinematic function matches the actual panda robot data. However, Jacobian's derivative and Jacobian have some errors.


## Function 
- real_data : Franka emika panda robot actual data files

- model : Franka emika panda robot model library folder

  + get_CoriolisMatrix.m : Coriolis matrix function <img src="https://render.githubusercontent.com/render/math?math=\C(\q,\dot{\q})&mode=inline"> [1]
  + get_CoriolisVector.m : Coriolis vector function <img src="https://render.githubusercontent.com/render/math?math=\C(\q,\dot{\q})\dot{\q}&mode=inline"> [1] 
  + get_FrictionTorque.m : Friction torque function <img src="https://render.githubusercontent.com/render/math?math=\F(\dot{\q})&mode=inline"> [1]
  + get_GravityVector.m : Gravity vector function <img src="https://render.githubusercontent.com/render/math?math=\G(\q)&mode=inline"> [1]
  + get_MassMatrix.m : Inertia matrix function <img src="https://render.githubusercontent.com/render/math?math=\M(\q)&mode=inline">[1]
  + get_Jacobian_dot.m : Jacobian Differential Functions <img src="https://render.githubusercontent.com/render/math?math=\dot{\J}(\q,\dot{\q})&mode=inline">
  + get_JacobianZ2YZ1.m :  Jacobian function <img src="https://render.githubusercontent.com/render/math?math=\J(\q)&mode=inline"> (direction z2-y-z1) 
  + get_pose.m :  kinematic function <img src="https://render.githubusercontent.com/render/math?math=\k(\q)&mode=inline">
  + plant.m :  robot manipulator plant function <img src="https://render.githubusercontent.com/render/math?math=\ddot{\q}%20=%20\M^{-1}(\q)(-\C(\q,\dot{\q})\dot{\q}-\G(\q)-\F(\dot{\q})%2B\boldsymbol\tau)&mode=inline">
  + simple_plant.m :  robot manipulator plant 함수 <img src="https://render.githubusercontent.com/render/math?math=\ddot{\q}%20=%20\M^{-1}(\q)(\boldsymbol\tau)&mode=inline">
  + rk.m /simple_rk.m :  Runge-Kutta 함수

- 0. test_model_kinematics.m : kinematics test code

- 0. test_model_jacobain.m : jacobian test Code

- 0. test_model_dot_jacobain.m : jacobian dot  test Code

- 1. main_cartesian_pd_control_playback.m : Cartesian PD controller code

- 1. main_impedance_control.m : Cartesian Impedance Controller Code

- 1. main_simple_pd_control_playback.m : Simple PD controller code

## Controller
- simple pd controller

<img src="./fig/control_simplePD_playback.png" alt="control_simplePD_playback"  width="375" /> 
- catesian pd controller

<img src="./fig/control_catesianPD_playback.png" alt="control_catesianPD_playback"  width="375" /> 
- position based impendace controller

<img src="./fig/control_catesian_imp.png" alt="control_catesian_imp"  width="375" /> 


 ## Kinematics Test

-  Franka Emika Robot DH parameters

|   i   |   1   |   2   |   3   |   4    |    5    |   6   |   7   |   8   |
| :---: | :---: | :---: | :---: | :----: | :-----: | :---: | :---: | :---: |
| theta |  q1   |  q2   |  q3   |   q4   |   q5    |  q6   |  q7   |   0   |
|   d   | 0.333 | 0.000 | 0.316 | 0.000  |  0.384  | 0.000 | 0.000 | 0.107 |
|   a   | 0.000 | 0.000 | 0.000 | 0.0825 | -0.0825 | 0.000 | 0.088 | 0.000 |
| alpha |   0   | -pi/2 | pi/2  |  pi/2  |  -pi/2  | pi/2  | pi/2  |   0   |

- Modified DH parameters
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%7B%7D+%5E%7Bi-1%7D+T_%7Bi%7D+%3D+%5Ctext%7Brot%7D_%7Bx%2C%5Calpha_%7Bi-1%7D%7D+%5Ctext%7Btrans%7D_%7Bx%2Ca_%7Bi-1%7D%7D%5Ctext%7Brot%7D_%7Bz%2C%5Cthata_%7Bi%7D%7D+%5Ctext%7Btrans%7D_%7Bz%2Cd_%7Bi%7D%7D+" 
alt="{} ^{i-1} T_{i} = \text{rot}_{x,\alpha_{i-1}} \text{trans}_{x,a_{i-1}}\text{rot}_{z,\thata_{i}} \text{trans}_{z,d_{i}} ">

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%7B%7D+%5E%7Bi-1%7D+T_%7Bi%7D+%3D+%5Cleft%5B%7B%5Cbegin%7Barray%7D%7Bccc%7Cc%7D%5Ccos+%5Ctheta+_%7Bi%7D+%26+-%5Csin+%5Ctheta+_%7Bi%7D+%26+0+%26+a_%7Bi-1%7D%5C%5C+%5Csin+%5Ctheta+_%7Bi%7D%5Ccos+%5Calpha+_%7Bi-1%7D+%26+%5Ccos+%5Ctheta+_%7Bi%7D%5Ccos+%5Calpha+_%7Bi-1%7D+%26+-%5Csin+%5Calpha+_%7Bi-1%7D+%26+-d_%7Bi%7D%5Csin+%5Calpha+_%7Bi-1%7D%5C%5C%5Csin+%5Ctheta+_%7Bi%7D%5Csin+%5Calpha+_%7Bi-1%7D+%26+%5Ccos+%5Ctheta+_%7Bi%7D%5Csin+%5Calpha+_%7Bi-1%7D+%26+%5Ccos+%5Calpha+_%7Bi-1%7D+%26+d_%7Bi%7D%5Ccos+%5Calpha+_%7Bi-1%7D%5C%5C%5Chline+0+%26+0+%26+0+%26+1%5Cend%7Barray%7D%7D%5Cright%5D" 
alt="{} ^{i-1} T_{i} = \left[{\begin{array}{ccc|c}\cos \theta _{i} & -\sin \theta _{i} & 0 & a_{i-1}\\ \sin \theta _{i}\cos \alpha _{i-1} & \cos \theta _{i}\cos \alpha _{i-1} & -\sin \alpha _{i-1} & -d_{i}\sin \alpha _{i-1}\\\sin \theta _{i}\sin \alpha _{i-1} & \cos \theta _{i}\sin \alpha _{i-1} & \cos \alpha _{i-1} & d_{i}\cos \alpha _{i-1}\\\hline 0 & 0 & 0 & 1\end{array}}\right]">

```matlab
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
```

- kinematics test plot

<img src="./fig/kin_test1.png" alt="kin_test1"  width="375" />  <img src="./fig/kin_test2.png" alt="kin_test2" width="375" />


 ##  Jacobian Test

- Jacobian test plot

  > Cartesian derivative(real) :  <img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cdot%7Bx%7D+%3D+J%28q%29%5Cdot%7Bq%7D" 
alt="\dot{x} = J(q)\dot{q}">
  >
  > - data1: <img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cdot%7Bx%7D+%3D+%28k%28q%29_%7Bt%2B1%7D-k%28q%29_%7Bt%7D%29%5CDelta+t" 
alt="\dot{x} = (k(q)_{t+1}-k(q)_{t})\Delta t">
  > - data2:<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cdot%7Bx%7D+%3D+%28x_%7Bt%2B1%7D-x_%7Bt%7D%29%5CDelta+t" 
alt="\dot{x} = (x_{t+1}-x_{t})\Delta t">
  > - data3:<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cdot%7Bx%7D+%3D+%5Chat%7BJ%7D%28q%29%5Cdot%7Bq%7D" 
alt="\dot{x} = \hat{J}(q)\dot{q}">

- jacobain test plot

<img src="./fig/Jac_test_simple_test.png" alt="Jac_test_simple_test"  width="375" /> <img src="./fig/Jac_test_joint1~7 movement.png" alt="Jac_test_joint1~7 movement"  width="375" />

<img src="./fig/Jac_test_joint5 movement.png" alt="Jac_test_joint5 movement"  width="375" /> <img src="./fig/Jac_test_joint7 movement.png" alt="Jac_test_joint7 movement"  width="375" />

## jacobian derivative
- Jacobian derivative test plot

  > Cartesian second derivative(real) : <img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cddot%7Bx%7D+%3D+%28%5Cdot%7Bx%7D_%7Bt%2B1%7D-%5Cdot%7Bx%7D_%7Bt%7D%29%5CDelta+t" 
alt="\ddot{x} = (\dot{x}_{t+1}-\dot{x}_{t})\Delta t">
  > - com: <img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cddot%7Bx%7D+%3D+J%28q%29%5Cddot%7Bq%7D+%2B%5Cdot%7BJ%7D%28q%2C%5Cdot%7Bq%7D%29%5Cdot%7Bq%7D" 
alt="\ddot{x} = J(q)\ddot{q} +\dot{J}(q,\dot{q})\dot{q}">

- jacobain dot test plot

<img src="./fig/Jac_dot_test_simple_test.png" alt="Jac_dot_test_simple_test"  width="375" /> <img src="./fig/Jac_dot_test_joint1~7 movement.png" alt="Jac_dot_test_joint1~7 movement"  width="375" />

<img src="./fig/Jac_dot_test_joint5 movement.png" alt="Jac_dot_test_joint5 movement"  width="375" /> <img src="./fig/Jac_dot_test_joint7 movement.png" alt="Jac_dot_test_joint7 movement"  width="375" />





> [1] *Gaz, Claudio, et al. "Dynamic identification of the* *franka* *emika* *panda robot with retrieval of feasible parameters using penalty-based optimization." IEEE Robotics and Automation Letters 4.4 (2019): 4147-4154.*
>
> (*https://github.com/marcocognetti/FrankaEmikaPandaDynModel*)
