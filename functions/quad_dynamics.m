function dstate = quad_dynamics(~, state, u, p)
% QUAD_DYNAMICS  Nonlinear 6-DOF rigid-body dynamics of a quadrotor.
%
%   dstate = quad_dynamics(t, state, u, p)
%
%   state = [x y z xdot ydot zdot phi theta psi p_rate q_rate r_rate]'
%   u     = [T tau_phi tau_theta tau_psi]'  (total thrust & body torques)
%   p     = parameter struct from quad_params()
%
%   Frame convention: Z-axis points UP in the inertial frame (ENU-style),
%   ZYX Euler angles (yaw-pitch-roll) define body attitude.

    % --- unpack state ---
    phi   = state(7);
    theta = state(8);
    psi   = state(9);
    pr    = state(10);   % body roll rate
    qr    = state(11);   % body pitch rate
    rr    = state(12);   % body yaw rate

    % --- unpack control ---
    T        = u(1);
    tau_phi  = u(2);
    tau_theta= u(3);
    tau_psi  = u(4);

    % --- translational acceleration (thrust rotated into inertial frame) ---
    xdd = (T/p.m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    ydd = (T/p.m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    zdd = (T/p.m) * (cos(phi)*cos(theta)) - p.g;

    % --- rotational acceleration (Euler's equations, body frame) ---
    pdot = ((p.Iyy - p.Izz)/p.Ixx) * qr * rr + tau_phi   / p.Ixx;
    qdot = ((p.Izz - p.Ixx)/p.Iyy) * pr * rr + tau_theta / p.Iyy;
    rdot = ((p.Ixx - p.Iyy)/p.Izz) * pr * qr + tau_psi   / p.Izz;

    % --- Euler angle rates from body rates ---
    phidot   = pr + sin(phi)*tan(theta)*qr + cos(phi)*tan(theta)*rr;
    thetadot = cos(phi)*qr - sin(phi)*rr;
    psidot   = (sin(phi)/cos(theta))*qr + (cos(phi)/cos(theta))*rr;

    dstate = zeros(12,1);
    dstate(1:3)  = state(4:6);          % position derivative = velocity
    dstate(4)    = xdd;
    dstate(5)    = ydd;
    dstate(6)    = zdd;
    dstate(7)    = phidot;
    dstate(8)    = thetadot;
    dstate(9)    = psidot;
    dstate(10)   = pdot;
    dstate(11)   = qdot;
    dstate(12)   = rdot;
end
