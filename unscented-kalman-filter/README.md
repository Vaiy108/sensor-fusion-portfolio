# Unscented Kalman Filter Sensor Fusion

## Overview
This project implements an Unscented Kalman Filter for tracking an object's state using lidar and radar measurements.  
The goal is to estimate the state of a moving object using nonlinear process and measurement models.

## Problem Statement
Estimate object position and motion from noisy nonlinear sensor measurements.

## Features
- Radar and lidar fusion
- CTRV motion model
- UKF prediction and update steps
- C++ implementation with CMake

## Algorithm Overview

The Unscented Kalman Filter is used to estimate the state vector:

x = [px, py, v, yaw, yaw_rate]

The filter performs two main steps:

### Prediction
1. Generate sigma points
2. Augment sigma points with process noise
3. Predict sigma points through the motion model
4. Compute predicted state mean and covariance

### Update

For each sensor measurement:

**Lidar**
- Linear position update

**Radar**
- Nonlinear measurement update using polar coordinates

Radar and lidar measurements are fused to improve state estimation.

## Dependencies

- C++
- CMake
- Eigen3

Install Eigen:

Ubuntu:

sudo apt install libeigen3-dev

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```
## Run the program

./ukf

## Project Structure
```
unscented-kalman-filter
├── src
│ ├── main.cpp
│ ├── ukf.cpp
│ ├── ukf.h
│ └── tools.cpp
├── media
├── CMakeLists.txt
└── README.md
```
## Results
The filter estimates object position and velocity using radar and lidar sensor fusion.

### Tracking Visualization
![UKF tracking result](media/ukf_tracking.png)

## Notes
In this version, Eigen is resolved via `find_package(Eigen3 CONFIG REQUIRED)` instead of bundling the dependency in the repository.
