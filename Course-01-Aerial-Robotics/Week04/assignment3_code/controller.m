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
g = params.gravity;
m = params.mass;
Ixx = params.I;

minF = params.minF;
maxF = params.maxF;

% Position control
K_p_1 = 12;
K_d_1 = 3;
K_p_2 = 12;
K_d_2 = 3;
K_p_3 = 12;
K_d_3 = 3;

r3_T_ddot = des_state.acc(3);
r3_T_dot = des_state.vel(3);
r3_T = des_state.pos(3);
r3_dot = state.vel(3);
r3 = state.pos(3);
% Thrust
r3_des_ddot = r3_T_ddot ...
            + K_d_3*(r3_T_dot - r3_dot) ...
            + K_p_3*(r3_T - r3);% (11)
F = m*g + m*r3_des_ddot; % (12)
if F < minF
    F = minF;
elseif F > maxF
    F = maxF;
end

r1_T_ddot = des_state.acc(1);
r1_T_dot = des_state.vel(1);
r1_T = des_state.pos(1);
r1_dot = state.vel(1);
r1 = state.pos(1);
r1_des_ddot = r1_T_ddot ...
            + K_d_1*(r1_T_dot - r1_dot) ...
            + K_p_1*(r1_T - r1);% (11)

r2_T_ddot = des_state.acc(2);
r2_T_dot = des_state.vel(2);
r2_T = des_state.pos(2);
r2_dot = state.vel(2);
r2 = state.pos(2);
r2_des_ddot = r2_T_ddot ...
            + K_d_2*(r2_T_dot - r2_dot) ...
            + K_p_2*(r2_T - r2);% (11)
        
phi = state.rot(1); 
theta = state.rot(2); 
psi = state.rot(3); 
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);
% Other two outputs from position controller
phi_des = 1/g * (r1_des_ddot*sin(psi) - r2_des_ddot*cos(psi)); % (14a)
theta_des = 1/g * (r1_des_ddot*cos(psi) + r2_des_ddot*sin(psi)); % (14b)
psi_des = des_state.yaw; % (16a)

% Attitude control
K_p_phi = 48;
K_d_phi = 7;
K_p_theta = 48;
K_d_theta = 7;
K_p_psi = 20;
K_d_psi = 10;

p_des = 0; % (15a)
q_des = 0; % (15b)
r_des = des_state.yawdot; % (16b)



% Moment (10)
u2 = [K_p_phi*(phi_des - phi) + K_d_phi*(p_des - p);
      K_p_theta*(theta_des - theta) + K_d_theta*(q_des - q);
      K_p_psi*(psi_des - psi) + K_d_psi*(r_des - r);
     ];
M = Ixx * u2;
% =================== Your code ends here ===================

end
