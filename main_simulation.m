%% MAIN_SIMULATION
% Closed-loop simulation of a quadrotor drone tracking a rising helical
% trajectory using a cascaded PID controller (position -> attitude ->
% motor mixing -> nonlinear 6-DOF rigid body dynamics).
%
% Run this script in MATLAB (or GNU Octave) to simulate and visualize
% the drone's flight.

clear; clc; close all;
addpath('functions');

%% --- Setup ---
p  = quad_params();

dt      = 0.01;      % controller & integration step [s]
Tfinal  = 30;         % total simulation time [s]
tvec    = 0:dt:Tfinal;
N       = length(tvec);

% Initial state: [x y z xd yd zd phi theta psi p q r]
state = zeros(12,1);
state(1:3) = [1.5, 0, 1.0];   % start on the helix at t=0

ctrl_state.pos_int = [0;0;0];

% --- Logging arrays ---
log.t        = tvec;
log.state    = zeros(12, N);
log.ref_pos  = zeros(3, N);
log.u        = zeros(4, N);
log.omega    = zeros(4, N);

%% --- Simulation loop ---
for k = 1:N
    t = tvec(k);
    ref = get_reference(t);

    [u, ctrl_state] = quad_controller(state, ref, ctrl_state, dt, p);
    omega = motor_mixing(u, p);

    log.state(:,k)   = state;
    log.ref_pos(:,k) = ref.pos(:);
    log.u(:,k)       = u;
    log.omega(:,k)   = omega;

    if k < N
        % integrate one controller step with a fixed-control ODE solve
        [~, y] = ode45(@(tt,ss) quad_dynamics(tt, ss, u, p), [t, t+dt], state);
        state = y(end,:)';
    end
end

%% --- Tracking error metrics ---
pos_error = log.ref_pos - log.state(1:3,:);
rmse = sqrt(mean(sum(pos_error.^2,1)));
fprintf('Position tracking RMSE: %.4f m\n', rmse);

%% --- Plots: tracking performance ---
figure('Name','Position Tracking','Color','w','Position',[50 50 900 700]);
labels = {'X [m]','Y [m]','Z [m]'};
for i = 1:3
    subplot(3,1,i);
    plot(log.t, log.ref_pos(i,:), 'r--', 'LineWidth', 1.5); hold on;
    plot(log.t, log.state(i,:), 'b-', 'LineWidth', 1.2);
    ylabel(labels{i});
    if i == 1
        legend('Reference','Actual','Location','best');
        title('Drone Position Tracking (Reference vs Actual)');
    end
    grid on;
end
xlabel('Time [s]');
saveas(gcf, 'results/position_tracking.png');

figure('Name','Attitude & Motor Speeds','Color','w','Position',[980 50 900 700]);
subplot(2,1,1);
plot(log.t, rad2deg(log.state(7,:)), 'LineWidth',1.2); hold on;
plot(log.t, rad2deg(log.state(8,:)), 'LineWidth',1.2);
plot(log.t, rad2deg(log.state(9,:)), 'LineWidth',1.2);
legend('Roll \phi','Pitch \theta','Yaw \psi');
ylabel('Angle [deg]'); title('Attitude'); grid on;

subplot(2,1,2);
plot(log.t, log.omega', 'LineWidth', 1.0);
legend('Motor 1','Motor 2','Motor 3','Motor 4');
xlabel('Time [s]'); ylabel('\omega [rad/s]');
title('Rotor Speeds'); grid on;
saveas(gcf, 'results/attitude_motors.png');

figure('Name','3D Flight Path','Color','w');
plot3(log.ref_pos(1,:), log.ref_pos(2,:), log.ref_pos(3,:), 'r--','LineWidth',1.5); hold on;
plot3(log.state(1,:), log.state(2,:), log.state(3,:), 'b-','LineWidth',1.5);
grid on; axis equal; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Reference path','Drone trajectory');
title('3D Flight Path');
view(45,25);
saveas(gcf, 'results/flight_path_3d.png');

%% --- 3D animation (renders drone body + rotors flying the path) ---
animate_quadrotor(log, p, 'results/drone_animation.gif');

fprintf('Done. Plots and animation saved to the results/ folder.\n');
