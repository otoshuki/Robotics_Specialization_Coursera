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
% 
%           mass: 0.1800
%              I: [3x3 double]
%           invI: [3x3 double]
%        gravity: 9.8100
%     arm_length: 0.0860
%           minF: 0
%           maxF: 3.5316

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

Kd = 40;
Kp = 400;
Kd_rot = 2;
Kp_rot = 100;

err_pos = des_state.pos - state.pos;
err_vel = des_state.vel - state.vel;

rdes_ddot = des_state.acc + Kd*err_vel + Kp*err_pos;

g = params.gravity;
m = params.mass;

psi_des = des_state.yaw;
%Using the rotation matrix
phi_des = (rdes_ddot(1)*psi_des - rdes_ddot(2))/g;
theta_des = (rdes_ddot(1) + rdes_ddot(2)*psi_des)/g;
des_rot = [phi_des, theta_des, psi_des]';

err_rot = des_rot - state.rot;
err_omega = [0,0,des_state.yawdot]' - state.omega;

F = m*(g + rdes_ddot(3));

M = Kd_rot*err_omega + Kp_rot*err_rot;

% =================== Your code ends here ===================

end