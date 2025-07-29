# Multi-Sensor Fusion for Mobile Robot Localization (ROS2)

## Overview
This project implements a **multi-sensor fusion pipeline** for mobile robots using the 
[`robot_localization`](https://github.com/cra-ros-pkg/robot_localization) package in ROS2.
It fuses multiple sensor sources (wheel odometry, IMU, GPS, and optional visual odometry)
to produce a robust estimate of the robot's pose, velocity, and acceleration, 
improving localization stability for both indoor and outdoor navigation.

---

## Features
- Fuses **wheel odometry** and **IMU** for stable indoor localization.
- Adds **GPS** (NavSat) for outdoor localization.
- (Optional) Adds **visual odometry** for drift reduction in GPS-denied environments.
- Generates a consistent `/odometry/filtered` topic for use in:
  - SLAM (e.g., SLAM Toolbox, RTAB-Map)
  - AMCL-based navigation (Nav2)
- Provides ready-to-use **YAML configurations** for indoor and outdoor modes.
- Includes **launch files** for quick startup.
- Provides **evaluation scripts** (ATE/RPE) using `evo` for trajectory accuracy analysis.

---

## Repository Structure
