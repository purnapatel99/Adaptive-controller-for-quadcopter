function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 traj t1
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];

waypoints0 = waypoints;
waypoints0 = transpose(waypoints0);
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);
    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
       scale = t/d0(t_index-1);
%        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
   % desired_state.vel = zeros(3,1);
   % desired_state.acc = zeros(3,1);
   % desired_state.yaw = 0;
   % desired_state.yawdot = 0;




%% Fill in your code here
s=size(waypoints0);
traj=zeros((s(1,1)-1),3);
traj_vel=zeros((s(1,1)-1),3);
traj_acc=zeros((s(1,1)-1),3);
des_t=zeros((s(1,1)-1),1);
%ans2=zeros((s(1,1)-1),1);
for a=1:(s(1,1)-1)    
    T=2*(sqrt((waypoints0(a+1,1)-waypoints0(a,1))^2+(waypoints0(a+1,2)-waypoints0(a,2))^2+(waypoints0(a+1,3)-waypoints0(a,3))^2));
    %ans2=t
   
    A=[0 0 0 0 0 1;
        T^5 T^4 T^3 T^2 T 1;
        0 0 0 0 1 0;
        5*T^4 4*T^3 3*T^2 2*T 1 0;
        0 0 0 2 0 0;
        20*T^3 12*T^2 6*T 2 0 0];
        
    for b=1:3
        if (a==1)
            vi=0;
        else
            vi=(waypoints0(a+1,b)-waypoints0(a,b))/(T);
        end
        if (a==s(1,1)-1)
            vf=0;
        else
            vf=(waypoints0(a+2,b)-waypoints0(a+1,b))/(T);
        end
        D=[waypoints0(a,b);waypoints0(a+1,b);vi;vf;0;0];
        C=inv(A)*D;
        %if (a==1)
        traj(a,b)=C(1,1)*t^5+C(2,1)*t^4+C(3,1)*t^3+C(4,1)*t^2+C(5,1)*t+C(6,1);
        traj_vel(a,b)=C(1,1)*5*t^4+C(2,1)*4*t^3+C(3,1)*3*t^2+C(4,1)*2*t+C(5,1);
        traj_acc(a,b)=C(1,1)*20*t^3+C(2,1)*12*t^2+C(3,1)*6*t+C(4,1)*2;
       % else
       % traj(a,b)=C(1,1)*(t-des_t(a-1,1))^5+C(2,1)*(t-des_t(a-1,1))^4+C(3,1)*(t-des_t(a-1,1))^3+C(4,1)*(t-des_t(a-1,1))^2+C(5,1)*(t-des_t(a-1,1))+C(6,1);
       % traj_vel(a,b)=C(1,1)*5*(t-des_t(a-1,1))^4+C(2,1)*4*(t-des_t(a-1,1))^3+C(3,1)*3*(t-des_t(a-1,1))^2+C(4,1)*2*(t-des_t(a-1,1))+C(5,1);
       % traj_acc(a,b)=C(1,1)*20*(t-des_t(a-1,1))^3+C(2,1)*12*(t-des_t(a-1,1))^2+C(3,1)*6*(t-des_t(a-1,1))+C(4,1)*2;
        
        %end
    end
end
%dist=sqrt((waypoints0(2,1)-state.pos(1,1))^2+(waypoints0(2,2)-state.pos(2,1))^2+(waypoints0(2,3)-state.pos(3,1))^2);
%e=0;
%for e=1:(s(1,1)-1)
    %d=sqrt((waypoints0(2,1)-state.pos(1,1))^2+(waypoints0(2,2)-state.pos(2,1))^2+(waypoints0(2,3)-state.pos(3,1))^2);
    if (t_index<=2)
        desired_state.pos = [traj(1,1);traj(1,2);traj(1,3)];
        desired_state.vel = [traj_vel(1,1);traj_vel(1,2);traj_vel(1,3)];
        desired_state.acc = [traj_acc(1,1);traj_acc(1,2);traj_acc(1,3)];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    %else
    %    d=sqrt((waypoints0(3,1)-state.pos(1,1))^2+(waypoints0(3,2)-state.pos(2,1))^2+(waypoints0(3,3)-state.pos(3,1))^2);
    %end 
        
    elseif (t_index<=3)
        desired_state.pos = [traj(2,1);traj(2,2);traj(2,3)];
        desired_state.vel = [traj_vel(2,1);traj_vel(2,2);traj_vel(2,3)];
        desired_state.acc = [traj_acc(2,1);traj_acc(2,2);traj_acc(2,3)];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    %else
    %d=sqrt((waypoints0(4,1)-state.pos(1,1))^2+(waypoints0(4,2)-state.pos(2,1))^2+(waypoints0(4,3)-state.pos(3,1))^2);    
    %end
    elseif (t_index<=4)
        desired_state.pos = [traj(3,1);traj(3,2);traj(3,3)];
        desired_state.vel = [traj_vel(3,1);traj_vel(3,2);traj_vel(3,3)];
        desired_state.acc = [traj_acc(3,1);traj_acc(3,2);traj_acc(3,3)];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    %else
    %d=sqrt((waypoints0(5,1)-state.pos(1,1))^2+(waypoints0(5,2)-state.pos(2,1))^2+(waypoints0(5,3)-state.pos(3,1))^2);    
    %end
    else
        desired_state.pos = [traj(4,1);traj(4,2);traj(4,3)];
        desired_state.vel = [traj_vel(4,1);traj_vel(4,2);traj_vel(4,3)];
        desired_state.acc = [traj_acc(4,1);traj_acc(4,2);traj_acc(4,3)];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
%end
end
end

