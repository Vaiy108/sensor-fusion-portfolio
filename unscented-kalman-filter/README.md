# Unscented Kalman Filter Sensor Fusion

This project implements an Unscented Kalman Filter (UKF) for sensor fusion using radar and lidar measurements.  
The goal is to estimate the state of a moving object using nonlinear process and measurement models.

This project was completed as part of the Udacity Self-Driving Car Engineer Nanodegree.

## Algorithm Overview

The Unscented Kalman Filter is used to estimate the state vector:

x = [px, py, v, yaw, yaw_rate]

The filter performs two main steps:

1. Prediction
2. Measurement Update

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
Run the program

./ukf

Project Śtructure

unscented-kalman-filter
│
├── src
│   ├── ukf.cpp
│   ├── ukf.h
│   ├── main.cpp
│   └── tools.cpp
│
├── media
│
├── CMakeLists.txt
└── README.md

Results
The filter estimates object position and velocity using radar and lidar sensor fusion.
