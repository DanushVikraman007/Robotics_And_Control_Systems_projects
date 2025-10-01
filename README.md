# CtrlRoboProjects

Welcome to **CtrlRoboProjects**, a collection of MATLAB projects exploring **control systems and robotics**. This repo includes both classic control projects and advanced autonomous vehicle simulations, and can easily grow with future projects.

---

## Current Projects

### 1. DC Motor Speed Control with PID

A simple and clear demonstration of **PID control** for a DC motor.

**Features:**

- Models a DC motor in MATLAB
- PID controller for speed regulation
- Step response simulation
- Plots motor speed and control input

### 2. Advanced MPC for Autonomous Vehicle Path Tracking
A Linear Time-Varying Model Predictive Controller (LTV-MPC) simulates an autonomous vehicle following a curved reference trajectory.
***Features:***

Kinematic bicycle model ([X; Y; θ] states)

Linearized time-varying MPC

Quadratic programming (QP) optimization (quadprog)

Steering constraints enforced

Logs vehicle states, steering commands, cross-track and heading errors

Dashboard plots:

Vehicle path vs reference

steering input

Cross-track error

Heading error

Requirements: MATLAB + Optimization Toolbox

### 3. 3-Link Robotic Arm Mouse-Follow Simulation

An interactive 3-link planar robotic arm simulation that follows mouse movement in real time, with trajectory logging.

*** Features: ***

Inverse kinematics for 3-link planar arm

Forward kinematics calculates end-effector position

Interactive mouse-following behavior

Logs and plots:

Target trajectory (mouse)

Actual end-effector trajectory

Joint angles θ1, θ2, θ3 vs time

End-effector linear/transverse velocities vs time
