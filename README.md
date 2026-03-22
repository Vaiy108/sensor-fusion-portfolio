# Sensor Fusion Portfolio

This repository showcases my work on **sensor fusion and state estimation** for autonomous systems, with a focus on **Kalman filtering techniques** used in robotics and self-driving applications.

All projects are implemented in **C++** with an emphasis on mathematical correctness, real-time considerations, and clear system design.

---

## 🚀 Featured Project: Unscented Kalman Filter (UKF)

A full implementation of an **Unscented Kalman Filter (UKF)** for fusing **radar and lidar measurements** to estimate the state of a moving object.

### 🔍 Problem
Estimate position, velocity, and orientation of an object using:
- **Radar (nonlinear measurements)** → range, bearing, range rate  
- **Lidar (linear measurements)** → position (x, y)

---

###  Approach

- State vector:  
  \[
  x = [p_x, p_y, v, \psi, \dot{\psi}]
  \]

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
<img src="media/ukf_track.gif" width="900"/>
</p>

- Accurate tracking of nonlinear motion
- Stable fusion of radar and lidar measurements
- Smooth trajectory estimation

```
> RMSE values:
> X: 0.06
> Y: 0.1009
> V_x: 0.45
> V_y: 0.63

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

### 💡 What I Learned

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
