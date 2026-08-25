function ref = get_reference(t)
% GET_REFERENCE  Desired trajectory for the drone to track: a rising helix.
%
%   ref = get_reference(t) returns a struct with fields:
%     pos [x y z], vel [xdot ydot zdot], yaw (rad)

    R      = 1.5;      % radius of the circle [m]
    w      = 0.5;       % angular speed [rad/s]
    climb  = 0.15;      % climb rate [m/s]
    z0     = 1.0;       % starting altitude [m]

    x  =  R*cos(w*t);
    y  =  R*sin(w*t);
    z  =  z0 + climb*t;

    xd = -R*w*sin(w*t);
    yd =  R*w*cos(w*t);
    zd =  climb;

    ref.pos = [x, y, z];
    ref.vel = [xd, yd, zd];
    ref.yaw = 0;   % keep nose pointing along +x (no yaw tracking in this demo)
end
