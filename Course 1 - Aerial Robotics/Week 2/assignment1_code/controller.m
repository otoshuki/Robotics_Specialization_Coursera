function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% FILL IN YOUR CODE HERE

%Taking in displacement
e = s_des(1) - s(1);

%Taking in velocity
e_dot = s_des(2) - s(2);

Kv = 30;
Kp = 200;

u =  params.mass * (1+Kv*e_dot + Kp*e + params.gravity);

end

