function p = quad_params()
% QUAD_PARAMS  Physical and controller parameters for the quadrotor drone.
%
% Returns a struct 'p' containing mass/inertia properties, geometry,
% motor/rotor coefficients, and PID gains used by the controller.

    % ---- Physical properties ----
    p.m   = 1.0;                 % mass [kg]
    p.g   = 9.81;                % gravity [m/s^2]
    p.l   = 0.225;                % arm length, center to rotor [m]
    p.Ixx = 0.0123;              % moment of inertia about x [kg m^2]
    p.Iyy = 0.0123;              % moment of inertia about y [kg m^2]
    p.Izz = 0.0224;              % moment of inertia about z [kg m^2]

    % ---- Rotor / motor mixing coefficients ----
    p.kF = 3.13e-5;              % thrust coefficient   T_i = kF * omega_i^2
    p.kM = 7.5e-7;               % drag/torque coeff.    M_i = kM * omega_i^2
    p.maxOmega = 838;            % max rotor speed [rad/s] (~8000 RPM)
    p.minOmega = 0;              % min rotor speed [rad/s]

    % ---- Outer loop (position) PID gains ----
    p.Kp_pos = [3.0, 3.0, 6.0];   % proportional [x y z]
    p.Ki_pos = [0.2, 0.2, 0.8];   % integral     [x y z]
    p.Kd_pos = [3.2, 3.2, 4.5];   % derivative   [x y z]

    % ---- Inner loop (attitude) PD gains ----
    p.Kp_att = [6.5, 6.5, 4.0];   % proportional [phi theta psi]
    p.Kd_att = [1.7, 1.7, 1.2];   % derivative   [phi theta psi]

    % ---- Yaw / max tilt limits ----
    p.maxTilt = deg2rad(30);      % maximum commanded roll/pitch [rad]
end
