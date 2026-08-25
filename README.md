Quadrotor Flight Simulation

A MATLAB/GNU Octave simulation of a quadrotor drone performing closed-loop trajectory tracking using a cascaded PID controller, nonlinear 6-DOF rigid-body dynamics, and motor mixing.

The simulation commands a rising helical trajectory, computes the required thrust and body torques, converts those commands into four rotor speeds, and integrates the quadrotor dynamics over a 30-second flight.

Overview

This project models the main computational components of a quadrotor flight-control system:

Nonlinear 6-DOF rigid-body dynamics

Cascaded position PID and attitude PD control

X-configuration motor mixing

Reference trajectory generation

Position and attitude tracking

Rotor-speed logging

3D flight-path visualization

Animated quadrotor visualization

Position-tracking RMSE calculation

System Workflow

Reference Trajectory
        │
        ▼
 Position PID Controller
        │
        ▼
Desired Acceleration
        │
        ├──► Desired Roll / Pitch
        │
        └──► Total Thrust
                 │
                 ▼
        Attitude PD Controller
                 │
                 ▼
        Desired Body Torques
                 │
                 ▼
           Motor Mixing
                 │
                 ▼
        Four Rotor Speeds
                 │
                 ▼
     Nonlinear Quadrotor Dynamics
                 │
                 ▼
          Updated 12-State
             Drone Model
                 │
                 └──────────► Feedback to Controller

Mathematical Model

The quadrotor is represented using a 12-state rigid-body model:

[x, y, z,
 x_dot, y_dot, z_dot,
 phi, theta, psi,
 p, q, r]

where:

x, y, z are inertial-frame positions

x_dot, y_dot, z_dot are linear velocities

phi, theta, psi are roll, pitch, and yaw angles

p, q, r are body-frame angular rates

The model uses a ZYX yaw-pitch-roll rotation convention and includes translational and rotational rigid-body dynamics.

Control Strategy

Outer Position Loop

The outer loop uses PID control to calculate the desired acceleration from position and velocity tracking errors.

Position Error
      +
Velocity Error
      ↓
   PID Control
      ↓
Desired Acceleration

The desired vertical acceleration is converted into total thrust with gravity compensation. Desired horizontal accelerations are mapped to roll and pitch commands.

Inner Attitude Loop

The inner loop uses PD control to track the desired roll, pitch, and yaw angles and generate the required body torques:

Desired Attitude
      -
Actual Attitude
      ↓
    PD Control
      ↓
Body Torques

Integral windup protection is included in the position controller by limiting the accumulated position error.

Motor Mixing

The controller output is:

[T, tau_phi, tau_theta, tau_psi]

where:

T = total thrust

tau_phi = roll torque

tau_theta = pitch torque

tau_psi = yaw torque

The motor mixer allocates these commands to four individual rotors using an X-configuration mixing matrix.

Rotor thrust follows:

f_i = kF * omega_i^2

where kF is the rotor thrust coefficient and omega_i is the rotor angular speed.

Rotor speeds are constrained to the configured physical limits.

Reference Trajectory

The default trajectory is a rising helix:

x(t) = R cos(wt)
y(t) = R sin(wt)
z(t) = z0 + climb*t

with:

Radius R = 1.5 m

Angular speed w = 0.5 rad/s

Climb rate 0.15 m/s

Initial altitude 1.0 m

Final simulation time 30 s

The trajectory is generated in get_reference.m, making it straightforward to replace the helix with another path.

Project Structure

QuadrotorDroneSim/
│
├── README.md
├── main_simulation.m
│
├── functions/
│   ├── quad_params.m
│   ├── quad_dynamics.m
│   ├── quad_controller.m
│   ├── motor_mixing.m
│   ├── get_reference.m
│   └── animate_quadrotor.m
│
└── results/
    ├── position_tracking.png
    ├── attitude_motors.png
    ├── flight_path_3d.png
    └── drone_animation.gif

File Descriptions

File

Purpose

main_simulation.m

Main simulation script, logging, plotting, and animation

quad_params.m

Defines drone physical parameters and controller gains

quad_dynamics.m

Computes nonlinear 6-DOF rigid-body dynamics

quad_controller.m

Implements cascaded position PID and attitude PD control

motor_mixing.m

Converts total thrust and torques into four rotor speeds

get_reference.m

Generates the desired rising-helical trajectory

animate_quadrotor.m

Generates the 3D quadrotor animation

results/

Contains generated plots and the animation

Simulation Configuration

The current simulation uses:

Parameter

Value

Mass

1.0 kg

Arm length

0.225 m

Gravity

9.81 m/s²

Simulation time

30 s

Integration/control step

0.01 s

Maximum rotor speed

838 rad/s

Maximum commanded tilt

30°

The physical parameters and PID/PD gains can be modified in functions/quad_params.m.

Requirements

MATLAB

A recent MATLAB installation with the standard numerical and plotting functionality is recommended.

GNU Octave

The project can also be run using GNU Octave. The animation function uses Octave's image package for GIF generation.

If required, install/load the package using:

pkg install -forge image
pkg load image

The project files use MATLAB-compatible syntax.

How to Run

Open MATLAB or GNU Octave.

Set the QuadrotorDroneSim folder as the current working directory.

Run:

main_simulation

The script will:

Load the quadrotor parameters.

Generate the reference trajectory.

Run the cascaded flight controller.

Perform motor mixing.

Integrate the nonlinear quadrotor dynamics.

Calculate the position-tracking RMSE.

Generate tracking and attitude plots.

Generate the 3D flight-path plot.

Generate the animated GIF.

Generated results are saved automatically in the results/ directory.

Results

The repository includes representative outputs from the simulation.

Position Tracking



The plot compares the desired and simulated X, Y, and Z positions over time.

Attitude and Rotor Speeds



This figure shows the simulated roll, pitch, and yaw angles together with the four rotor speeds.

3D Flight Path



The reference helical trajectory is compared with the simulated drone trajectory.

Quadrotor Animation



The animation visualizes the quadrotor body, rotor locations, and flight trail along the simulated trajectory.

Performance Metric

The simulation calculates the position-tracking RMSE using:

RMSE = sqrt(mean(||p_reference - p_actual||^2))

The value printed by the simulation should be treated as the result for the specific parameter configuration and simulation run.

Customization

The project can be modified in several places:

Change the trajectory

Edit:

functions/get_reference.m

The controller expects the reference generator to provide position, velocity, and yaw.

Change drone parameters

Edit:

functions/quad_params.m

This includes mass, inertia, arm length, rotor coefficients, rotor limits, and controller gains.

Change simulation duration or resolution

Edit the following values in:

main_simulation.m

dt = 0.01;
Tfinal = 30;

Limitations

This is a computational simulation rather than a complete physical flight model. The current implementation does not model effects such as:

Aerodynamic drag

Blade flapping

Motor electrical dynamics

Battery voltage variation

Wind disturbances

Ground effect

Sensor noise

Actuator delays

Full aerodynamic rotor interactions

The model is therefore intended primarily for studying quadrotor dynamics, trajectory tracking, and basic flight-control concepts.

Future Improvements

Possible extensions include:

Adding wind and external disturbances

Adding sensor noise and state estimation

Implementing a Kalman or complementary filter

Modeling motor dynamics

Comparing PID with LQR or nonlinear control

Adding trajectory planning for different paths

Adding controller performance comparisons

Testing robustness under parameter uncertainty

Author

RISHITHA C 

