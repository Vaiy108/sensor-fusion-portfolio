# Sensor Fusion Portfolio

## ⭐ Featured Project — Real-Time Radar-Camera Fusion (C++)

A real-time C++ prototype for multi-object tracking and sensor fusion using
camera detection, Kalman filtering, and EKF-based radar updates.

### What I implemented
- Real-time multi-object tracking in C++
- Kalman filter (constant velocity model)
- EKF radar update (range + angle)
- Camera-radar association
- Detection filtering and tracking stabilization

### Engineering Decisions
- Used motion detection for real-time performance on CPU
- Integrated YOLO as optional detector (higher accuracy, lower FPS)
- Used Kalman filter to maintain tracking between detections

### Key Features
- Motion-based object detection (real-time)
- Multi-object tracking with persistent IDs
- Constant-velocity Kalman filter
- EKF radar update (range + angle)
- Simulated radar measurements
- Optional YOLO detector (OpenCV DNN)

### Tech Stack
C++, OpenCV, CMake, Visual Studio

### Project repo Link
🔗 Repo: https://github.com/Vaiy108/radar-camera-fusion


---

## Core Filtering & Estimation Projects

### Unscented Kalman Filter (UKF)
Radar + Lidar fusion using CTRV motion model.

🔗 Project link: [unscented-kalman-filter](https://github.com/Vaiy108/sensor-fusion-portfolio/tree/main/unscented-kalman-filter)

Key concepts:
- Nonlinear state estimation
- Sigma points (unscented transform)
- Radar + lidar fusion

### Overview
Implemented a full Unscented Kalman Filter (UKF) to fuse radar and lidar measurements
for tracking a moving object under nonlinear dynamics (CTRV model). This project showcases my work on **sensor fusion and state estimation** for autonomous systems, with a focus on **Kalman filtering techniques** used in robotics and self-driving applications.

All projects are implemented in **C++** with an emphasis on mathematical correctness, real-time considerations, and clear system design.

---

###  Approach

- State vector:

$$
\[
x = [p_x, p_y, v, \psi, \dot{\psi}]
\]
$$


- Motion model:  
  **CTRV (Constant Turn Rate and Velocity)**

- Key components:
  - Augmented state with process noise
  - Sigma point generation using unscented transform
  - Nonlinear prediction step
  - Sensor-specific update steps:
    - Linear update (lidar)
    - Nonlinear update (radar)

---

### ⚙️ Implementation Highlights

- Full UKF pipeline:
  - Initialization from first measurement
  - Time-based prediction
  - Sensor fusion updates
- Angle normalization for stability
- Numerical safeguards (division-by-zero handling)
- Clean modular C++ design using Eigen

---

### 📊 Results

<p align="center">
<img src="media/ukf_track.gif" width="600"/>
</p>

- Accurate tracking of nonlinear motion
- Stable fusion of radar and lidar measurements
- Smooth trajectory estimation

- **RMSE:**
```
- px: 0.06
- py: 0.10
- vx: 0.42
- vy: 0.63
```
---

### 🛠️ Tech Stack

- C++
- Eigen (linear algebra)
- CMake
- Git/GitHub

---

### 📁 Project Structure
```
unscented-kalman-filter/
├── src/
│ ├── ukf.cpp
│ ├── ukf.h
│ └── main.cpp
├── media/
└── CMakeLists.txt
````


---

### 💡 Skills Learned

- Practical implementation of the **Unscented Kalman Filter**
- Handling nonlinear systems with sigma points
- Differences between radar and lidar measurement models
- Importance of numerical stability in estimation systems
- Real-world considerations in sensor fusion pipelines

---

## 🧩 Skills Demonstrated

- C++
- Sensor Fusion
- Kalman Filtering (EKF / UKF)
- Nonlinear State Estimation
- Unscented Transform
- Radar & Lidar Fusion
- Linear Algebra (Eigen)
- System Modeling (CTRV)

---

## 👤 Author

**Vasan Iyer**  
Sensor Fusion / Autonomous Systems Engineer  

Focus areas:
- Sensor fusion & state estimation  
- Autonomous systems  
- Flight dynamics & control  
- Embedded systems (C++, Python)  
- UAV systems & simulation  

GitHub: https://github.com/Vaiy108


---
