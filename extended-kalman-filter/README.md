# Extended Kalman Filter Sensor Fusion

## 🚀 Overview

This project implements an **Extended Kalman Filter (EKF)** for fusing **lidar and radar measurements** to estimate the state of a moving object. 

This project currently includes a small structured sample dataset to verify the EKF pipeline, sensor parsing, and RMSE evaluation flow. Final benchmark-quality performance evaluation should be performed on a larger dataset.

The implementation focuses on:
- nonlinear measurement handling through Jacobian-based linearization,
- sensor fusion across heterogeneous sensors,
- quantitative evaluation using ground-truth data and RMSE.

---

## Problem

Estimate the state of a moving object:
$$ 
\[
x = [p_x, p_y, v_x, v_y] 
\]
$$
using noisy sensor measurements from:

- **Lidar** → Cartesian position measurements \((p_x, p_y)\)
- **Radar** → Polar measurements \((\rho, \phi, \dot{\rho})\)

---

## Approach

The Extended Kalman Filter is used because radar measurements are nonlinear in Cartesian state space.

### EKF Pipeline
1. Initialize state from the first sensor measurement
2. Predict the next state using a constant-velocity motion model
3. Update with lidar using a linear measurement model
4. Update with radar using:
   - nonlinear measurement function
   - Jacobian matrix for linearization
   - angle normalization for bearing consistency

---

## 🔧 Design Decisions

- **State Representation**  
  The EKF uses a Cartesian velocity state:
  $$
  \[
  x = [p_x, p_y, v_x, v_y]
  \]
  $$

- **Motion Model**  
  A constant-velocity model is used for prediction.

- **Radar Update**  
  Radar measurements are transformed using a nonlinear measurement function and linearized using the Jacobian.

- **Lidar Update**  
  Lidar directly measures Cartesian position, so a standard linear Kalman update is used.

- **Numerical Stability**  
  Small-value safeguards are added to avoid division-by-zero issues in Jacobian and radar measurement calculations.

---

## Dataset and Evaluation

This EKF project uses a **structured measurement-log dataset**, where each line contains:

- a sensor type (`L` for lidar or `R` for radar),
- sensor measurements:
	- lidar measurements: position `(px, py)`
	- radar measurements: range, bearing, and range rate `(rho, phi, rho_dot)`
- a timestamp,
- ground-truth state values for RMSE evaluation

Example format:

```text
L p_x p_y timestamp gt_px gt_py gt_vx gt_vy
R rho phi rho_dot timestamp gt_px gt_py gt_vx gt_vy
```

This dataset format was chosen to evaluate the EKF quantitatively and compare estimation accuracy across sensor types. This dataset format allows:

- sequential prediction/update,
- direct comparison with ground truth,
- RMSE-based performance evaluation.

#### Why this differs from the UKF project

The UKF project in this portfolio uses a PCL-based simulation / point-cloud workflow and visualization pipeline, where measurements are generated within a simulated highway environment and visualized in 3D.

The EKF project uses a text-based sensor dataset instead of point-cloud simulation because the goal here is different and current dataset is a small sample for validation.

- UKF project → demonstrate simulation, visualization, and nonlinear filtering in a 3D environment
- EKF project → demonstrate filter design, Jacobian-based nonlinear handling, and quantitative evaluation with RMSE

This separation was intentional to show both:

- system-level simulation work,
- dataset-driven estimation and validation.

## 📊 Results

The filter is evaluated using Root Mean Square Error (RMSE) between the estimated state and ground truth.

Example reported metrics:
```
- px: 0.2341
- py: 0.2432
- vx: 1.4330
- vy: 1.5153
```
## My Contribution
- Implemented the full EKF pipeline in C++
- Designed lidar and radar update paths
- Implemented Jacobian-based radar linearization
- Added angle normalization and numerical stability safeguards
- Integrated RMSE-based evaluation with ground truth
- Structured the project for reproducible dataset-driven testing

## 🛠️ Tech Stack
- C++
- Eigen
- CMake

## 📂 Project Structure
```
extended-kalman-filter
├── src
│   ├── main.cpp
│   ├── ekf.cpp
│   ├── ekf.h
│   ├── measurement_package.h
│   ├── tools.cpp
│   └── tools.h
├── data
│   └── input.txt
├── CMakeLists.txt
└── README.md
````
## Build and Run
```
mkdir build
cd build
cmake ..
make
create extended-kalman-filter/data/input.txt
run
./ekf_demo ../data/input.txt
```
## Key Learnings
- Practical implementation of the Extended Kalman Filter
- Linearization of nonlinear radar measurements using the Jacobian
- Differences between EKF and UKF for sensor fusion
- Importance of numerical stability and angle normalization
- Value of dataset-driven validation using RMSE

## Author

Vasan Iyer
Sensor Fusion / Autonomous Systems Engineer

Focus:

- Sensor fusion & state estimation
- Autonomous systems
- Flight dynamics & control
- Embedded systems (C++, Python)
- UAV systems & simulation

GitHub: https://github.com/Vaiy108
