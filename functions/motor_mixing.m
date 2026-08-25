function omega = motor_mixing(u, p)
% MOTOR_MIXING  Convert [T tau_phi tau_theta tau_psi] into 4 rotor speeds
% for an X-configuration quadrotor (motors 1-4: front-right, rear-right,
% rear-left, front-left), then clip to physical rotor limits.
%
%   omega = motor_mixing(u, p)   returns 4x1 rotor speeds [rad/s]

    T        = u(1);
    tau_phi  = u(2);
    tau_theta= u(3);
    tau_psi  = u(4);

    l  = p.l;
    kF = p.kF;
    kM = p.kM;

    % Mixing matrix for X-configuration (thrust shares + differential arms)
    % f_i = kF * omega_i^2  (individual rotor thrust)
    A = [ 1,          1,          1,          1;
          l/sqrt(2), -l/sqrt(2), -l/sqrt(2),  l/sqrt(2);
         -l/sqrt(2), -l/sqrt(2),  l/sqrt(2),  l/sqrt(2);
          kM/kF,     -kM/kF,      kM/kF,     -kM/kF];

    f = A \ [T; tau_phi; tau_theta; tau_psi];   % individual rotor thrusts
    f = max(f, 0);                              % rotors can't pull

    omega = sqrt(f / kF);
    omega = min(max(omega, p.minOmega), p.maxOmega);
end
