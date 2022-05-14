close all;
clear;


addpath('utils');

global alpha_plot t_plot 
alpha_plot = [];
t_plot = [];
%% pre-calculated trajectories
%trajhandle = @traj_line;
trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
%  trajhandle = @traj_generator3;
%  waypoints = [0    0   0;
%               1    1   1;
%               2    0   2;
%               3    -1  1;
%               1    1   0];
%               1    0   0;
%               3    -1  0]';
%  trajhandle([],[],waypoints);

%% controller
controlhandle = @controller_adaptive;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);
