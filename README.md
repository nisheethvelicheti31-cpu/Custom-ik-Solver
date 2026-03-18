# 5-DOF Spatial Manipulator: Precision Control Interface
**Developed by Nisheeth Chowdary Velicheti** *Robotics Engineering, University of California, Santa Cruz*

![Custom-ik-Solver](https://github.com/user-attachments/assets/962f4a26-0cad-422b-b2b5-94d1c0247b8d)


## 📌 Overview
This project is a high-performance **3D Inverse Kinematics (IK) Solver** and **Teleoperation Dashboard**. It simulates a 5-Degree-of-Freedom (DOF) robotic arm capable of real-time end-effector tracking in $SE(3)$ space. The system utilizes a **Damped Least Squares (DLS)** Jacobian solver to ensure numerical stability and fluid motion, even near kinematic singularities.

## 🚀 Key Features
* **5-DOF Spatial Kinematics:** Full 3D reachability featuring a Base Yaw swivel and four pitching links.
* **Real-Time Teleoperation:** Interactive dashboard with sliders for sub-millimeter target precision.
* **Singularity Robustness:** Implementation of Damped Least Squares (DLS) to prevent mathematical "explosions" when the arm is fully extended.
* **Live Telemetry HUD:** Real-time tracking of Euclidean error and joint configurations (in degrees).
* **Object-Oriented Architecture:** Built with a modular Python class structure for high stability and low latency on macOS.

## 🧠 Mathematical Foundation

### 1. Forward Kinematics (FK)
The position of the end-effector $\vec{p}$ is calculated by accumulating the transformation matrices of each joint. For a given joint $i$ with length $L_i$ and pitch $\theta_i$ under a base yaw $\psi$:

$$x = \cos(\psi) \sum_{i=1}^{n} L_i \cos(\sum_{j=1}^{i} \theta_j)$$
$$y = \sin(\psi) \sum_{i=1}^{n} L_i \cos(\sum_{j=1}^{i} \theta_j)$$
$$z = \sum_{i=1}^{n} L_i \sin(\sum_{j=1}^{i} \theta_j)$$

### 2. The Jacobian Matrix ($J$)
To move the arm, we calculate the Jacobian, which maps joint velocities to Cartesian velocities. This is essential for converting desired hand movement into individual joint rotations:
$$J = \frac{\partial \vec{p}}{\partial \vec{\theta}}$$

### 3. Damped Least Squares (DLS)
To invert the Jacobian without instability at singularities, we use the DLS method (Levenberg-Marquardt):
$$\Delta \theta = J^T (J J^T + \lambda^2 I)^{-1} \vec{e}$$

## 🛠️ Installation & Setup

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/nisheethvelicheti31-cpu/Custom-ik-Solver.git](https://github.com/nisheethvelicheti31-cpu/Custom-ik-Solver.git)
   cd Custom-ik-Solver
