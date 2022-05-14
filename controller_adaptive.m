function [F, M] = controller_adaptive(t, state, des_state, params)
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

global alpha_hat t_prev vel_prev omega_prev alpha_plot t_plot n

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
e = [des_state.pos(3,1)-state.pos(3,1);
     phi_c-state.rot(1,1);
     theta_c-state.rot(2,1);
     des_state.yaw-state.rot(3,1);
     des_state.vel(3,1)-state.vel(3,1);
     0-state.omega(1,1);
     0-state.omega(2,1);
     des_state.yawdot-state.omega(3,1)];
e = -e;
% Thrust
% F = params.mass*(params.gravity+R3c);
% 
% % Moment
% M = params.I*[M1;M2;M3];

% =================== Your code ends here ===================
 if t < 0.1
     m_hat = 0.05;
     Ixx_hat = 0.00015;
     Iyy_hat = 0.000332;
     Izz_hat = 0.0002728;
     r_hat = 0;
     alpha_hat = [m_hat; Ixx_hat; Iyy_hat; Izz_hat; m_hat*r_hat];
     t_prev = -0.1;
     vel_prev = [0;0;0];
     omega_prev = [0;0;0];
     n = 1;
 end
 
 acc = 1000*(state.vel - vel_prev)/(1000*(0.0001));
 acc_rot = 1000*(state.omega - omega_prev)/(1000*(0.0001));
 
% m = params.mass;
 g = params.gravity;
% r = 0;

% Ma = [m/(cos(state.rot(1,1))*cos(state.rot(2,1))) 0 0 0; [0;0;0] params.I];
% G =  [(g*m)/(cos(state.rot(1,1))*cos(state.rot(2,1))); 
%     -g*m*r*sin(state.rot(1,1));
%     -g*m*r*sin(state.rot(2,1));
%     0];
% 
% U = Ma*[R3c; M1; M2; M3] + G;

Y_out = [(R3c + g)/(cos(state.rot(1,1))*cos(state.rot(2,1))) 0 0 0 0;
        0 M1 0 0 -g*sin(state.rot(1,1));
        0 0 M2 0 -g*sin(state.rot(2,1));
        0 0 0 M3 0];
U = Y_out*alpha_hat;

Y = [(acc(3,1) + g)/(cos(state.rot(1,1))*cos(state.rot(2,1))) 0 0 0 0;
        0 acc_rot(1,1) 0 0 -g*sin(state.rot(1,1));
        0 0 acc_rot(2,1) 0 -g*sin(state.rot(2,1));
        0 0 0 acc_rot(3,1) 0];
M_hat = [alpha_hat(1)/(cos(state.rot(1,1))*cos(state.rot(2,1))) 0 0 0;
        0 alpha_hat(2) 0 0;
        0 0 alpha_hat(3) 0
        0 0 0 alpha_hat(4)];
phi = M_hat\Y;
KP = -diag([Kp3;Kpphi;Kptheta;Kpsi])*eye(4);
KD = -diag([Kd3;Kdphi;Kdtheta;Kdsi])*eye(4);
Acl = [zeros(4,4), eye(4);
        KP, KD];
B = [zeros(4,4); eye(4)];

Q = diag([25;1;1;0.1;8;1;1;0.1])*eye(8);
P = lyap(Acl', Q);

gamma = eye(5).*305; 

d_alpha_hat = -gamma\(phi'*B'*P*e);

% [tm,y] = ode45(@(tm,y) d_alpha_hat, [t-0.001 t], alpha_hat);
% b = size(y);
% alpha_hat = y(b(1), :)';

alpha_hat = alpha_hat + d_alpha_hat*(0.0001);
alpha_plot(n, :) = alpha_hat';
t_plot(n) = t;
% t - t_prev
t_prev = t;
vel_prev = state.vel;
omega_prev = state.omega;
n = n+1;

% pause(0.001);

F = U(1);

M = [U(2);U(3);U(4)];
end
