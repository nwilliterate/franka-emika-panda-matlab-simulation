# franka-matlab-simulation

└ real_data: 판다 7축 실제 수집 데이터 파일
└ model : 판다 7축 모델 관련 함수 파일
   └ get_CoriolisMatrix.m     : 코리올리 매트릭스 함수
   └ get_CoriolisVector.m     : 코리올리 벡터 함수
   └ get_FrictionTorque.m    : 마찰력 토크 함수
   └ get_GravityVector.m      : 중력 함수
   └ get_Jacobian_dot.m      : 자코비안 미분 함수
   └ get_JacobianZ2YZ1.m   : 자코비안 함수
   └ get_MassMatrix.m         : 이너셔 매트릭스 함수
   └ get_pose.m                    :  기구학 함수  
   └ plant.m                           :  robot manipulator plant 함수 (ddq = inv(m)*(-c-G-F+u))
   └ rk.m /simple_rk.m          :  Runge-Kutta 함수
   └ simple_plant.m               :  robot manipulator plant 함수 (ddq = inv(m)*(u))
└ main_admittance_control(not complete).m : 판다 7축 로봇 어드미턴스 제어기 코드(미완성)
└ main_cartesian_pd_control.m      : 판다 7축 로봇 카테시안 pd 제어기 코드
└ main_impedance_control.m         : 판다 7축 로봇 임피던스 제어기 코드
└ main_simple_pd_control.m          : 판다 7축 로봇 간단한 pd 제어기 코드
