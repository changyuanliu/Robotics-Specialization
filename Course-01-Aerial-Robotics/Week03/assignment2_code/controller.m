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
g = params.gravity;
m = params.mass;
Ixx = params.Ixx;
minF = params.minF;
maxF = params.maxF;

K_pz = 160;
K_dz = 20;
K_pphi = 40;
K_dphi = 2;
K_py = 80;
K_dy = 10;

phi_c = -1/g*( des_state.acc(1) + K_dy*(des_state.vel(1)-state.vel(1)) + K_py*(des_state.pos(1)-state.pos(1)));
u1 = m * (g +  des_state.acc(2) + K_dz*(des_state.vel(2)-state.vel(2)) + K_pz*(des_state.pos(2)-state.pos(2)));
if u1 < minF
    u1 = minF;
elseif u1 > maxF
    u1 = maxF;
end       
u2 = (K_pphi*(phi_c - state.rot) + K_dphi*(0 - state.omega));
% if u2 < minF
%     u2 = minF;
% elseif u2 > maxF
%     u2 = maxF;
% end   
% FILL IN YOUR CODE HERE

end

