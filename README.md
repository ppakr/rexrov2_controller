# rexrov2_controller

## Overview

`rexrov2_controller` is a control package for AUV using PID controllers for linear and angular control.

## Features

- PID control for linear and angular motion
- Configurable PID gains
- Control callbacks for real-time adjustments

## Configuration

The following parameters can be configured in the `rexrov2_control.py` script:

- `control_rate`: The rate at which the control algorithm runs (in Hz)
- PID gains for linear and angular control:
  - `k_p_u`, `k_i_u`, `k_d_u`: PID gains for linear control (X axis)
  - `k_p_yaw`, `k_i_yaw`, `k_d_yaw`: PID gains for angular control (Z axis)
