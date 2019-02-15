function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% FILL IN YOUR CODE HERE
K_dy = 10;
K_py = 100;
K_dz = 5000;
K_pz = 5;
K_dphi = 50;
K_pphi = 1000;

%phi_c = -1/g * (ydes.. + kdy(ydes. - y.) + kpy(yde-y))
phi_c = -(1/params.gravity)*(des_state.acc(1) + K_dy*(des_state.vel(1) - state.vel(1))...
    + K_py*(des_state.pos(1) - state.pos(1)));
phi_dot_c = 0;
phi_ddot_c = 0;

%u2 = Ixx (phi.. + kdphi(phic. - phi.) + kpphi(phic - phi))
u2 = params.Ixx * (phi_ddot_c + K_dphi*(phi_dot_c - state.omega) + K_pphi*(phi_c - state.rot));

%u1 = m(g + z.. + kdz(zdes. - z.) + kpz(zdes - z))
u1 = params.mass * (params.gravity + des_state.acc(2) + K_dz*(des_state.vel(2) - state.vel(2))...
    + K_pz*(des_state.pos(2) * state.pos(2)));

end

