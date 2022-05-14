function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
Kd3=15;
Kp3=150;
Kd1=8;
Kp1=8;
Kd2=8;
Kp2=8;
R3c=des_state.acc(3,1)+Kd3*(des_state.vel(3,1)-state.vel(3,1))+Kp3*(des_state.pos(3,1)-state.pos(3,1));
R1c=des_state.acc(1,1)+Kd1*(des_state.vel(1,1)-state.vel(1,1))+Kp1*(des_state.pos(1,1)-state.pos(1,1));
R2c=des_state.acc(2,1)+Kd2*(des_state.vel(2,1)-state.vel(2,1))+Kp2*(des_state.pos(2,1)-state.pos(2,1));

phi_c= atan2((R1c*sin(des_state.yaw)-R2c*cos(des_state.yaw)), params.gravity);
theta_c= atan2((R1c*cos(des_state.yaw)+R2c*sin(des_state.yaw)), params.gravity);

Kpphi=150;
Kdphi=20;
Kptheta=150;
Kdtheta=20;
Kpsi=150;
Kdsi=20;
M1=Kpphi*(phi_c-state.rot(1,1))+Kdphi*(0-state.omega(1,1));
M2=Kptheta*(theta_c-state.rot(2,1))+Kdtheta*(0-state.omega(2,1));
M3=Kpsi*(des_state.yaw-state.rot(3,1))+Kdsi*(des_state.yawdot-state.omega(3,1));
% Thrust
% F = params.mass*(params.gravity+R3c);
% 
% % Moment
% M = params.I*[M1;M2;M3];

% =================== Your code ends here ===================

m = params.mass;
g = params.gravity;
r = 0;

Ma = [m/(cos(state.rot(1,1))*cos(state.rot(2,1))) 0 0 0; [0;0;0] params.I];
G =  [(g*m)/(cos(state.rot(1,1))*cos(state.rot(2,1))); 
    -g*m*r*sin(state.rot(1,1));
    -g*m*r*sin(state.rot(2,1));
    0];

U = Ma*[R3c; M1; M2; M3] + G;

F = U(1);

M = [U(2);U(3);U(4)];
end
