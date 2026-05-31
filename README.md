# Self-Balancing Robot Control using Event-Triggered LQR and Sliding Mode Control

This repository contains the modeling, simulation, and practical control implementation of a two-wheeled self-balancing robot using MATLAB and Simulink.

The main goal of this project is to compare conventional periodic control with event-triggered control strategies. In periodic control, the controller updates at every sampling instant. In event-triggered control, the controller updates only when a triggering condition is satisfied. This can reduce unnecessary control updates while keeping the robot balanced near the upright position.

---

## Project Overview

A self-balancing robot is an unstable nonlinear system similar to an inverted pendulum mounted on two wheels. Without feedback control, the robot falls away from the upright equilibrium. Therefore, a stabilizing controller is required.

The robot state is considered as:

```text
x = [gamma, theta, gamma_dot, theta_dot]
