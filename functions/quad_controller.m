function [u, ctrl_state] = quad_controller(state, ref, ctrl_state, dt, p)
% QUAD_CONTROLLER  Cascaded PID controller for quadrotor trajectory tracking.
%
%   [u, ctrl_state] = quad_controller(state, ref, ctrl_state, dt, p)
%
%   state      : current 12x1 state vector (see quad_dynamics)
%   ref        : struct with fields pos [x y z], vel [xd yd zd], yaw (rad)
%   ctrl_state : struct holding integral error terms (persists between calls)
%   dt         : controller time step [s]
%   p          : parameter struct from quad_params()
%
%   u = [T tau_phi tau_theta tau_psi]'

    % --- current position / velocity / attitude ---
    pos = state(1:3);
    vel = state(4:6);
    phi = state(7); theta = state(8); psi = state(9);
    rates = state(10:12);

    % =========================================================
    % OUTER LOOP: position PID -> desired acceleration -> desired
    % tilt angles (phi_des, theta_des) and total thrust T
    % =========================================================
    pos_err = ref.pos(:) - pos;
    vel_err = ref.vel(:) - vel;

    ctrl_state.pos_int = ctrl_state.pos_int + pos_err * dt;
    ctrl_state.pos_int = max(min(ctrl_state.pos_int, 2), -2); % anti-windup

    acc_des = p.Kp_pos(:).*pos_err + p.Ki_pos(:).*ctrl_state.pos_int + p.Kd_pos(:).*vel_err;

    ax_des = acc_des(1);
    ay_des = acc_des(2);
    az_des = acc_des(3);

    psi_des = ref.yaw;

    % Desired thrust (vertical channel, includes gravity compensation)
    T = p.m * (p.g + az_des);
    T = max(T, 0.1);   % keep thrust positive

    % Map desired horizontal accel to desired roll/pitch (small-angle inverse)
    phi_des   = (1/p.g) * (ax_des*sin(psi_des) - ay_des*cos(psi_des));
    theta_des = (1/p.g) * (ax_des*cos(psi_des) + ay_des*sin(psi_des));

    % Saturate tilt commands
    phi_des   = max(min(phi_des,   p.maxTilt), -p.maxTilt);
    theta_des = max(min(theta_des, p.maxTilt), -p.maxTilt);

    % =========================================================
    % INNER LOOP: attitude PD -> body torques
    % =========================================================
    att_err  = [phi_des - phi; theta_des - theta; wrapToPi(psi_des - psi)];
    rate_err = [0;0;0] - rates;   % desired body rates ~ 0 (simple regulation)

    tau = p.Kp_att(:).*att_err + p.Kd_att(:).*rate_err;

    u = [T; tau(1); tau(2); tau(3)];
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
