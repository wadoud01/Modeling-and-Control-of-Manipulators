# Modelling and Control of Manipulator (MCM) Project

This repository contains MATLAB implementations for Modelling and Control of Manipulator Project. The project involves the analysis, simulation, and control of a 7-link robotic manipulator, with tasks including direct kinematics, Jacobian computation, and inverse kinematics.

Results could be found in the end.

---

## Features

### Assignment 2: Manipulator Geometry and Direct Kinematics
This part focuses on:

- **Geometry Definition:** Defining the manipulator model using transformation matrices.
- **Direct Kinematics:** Computing the position and orientation of each link in the manipulator.
- **Visualization:** Animating the robot's movement for different joint configurations.

#### Key Highlights:
1. Transformation matrices calculated for each link relative to the base.
2. Direct geometry computed for multiple configurations.
3. Animated trajectories illustrating the motion of the manipulator in 3D space.

#### Results:
- Plots demonstrating the position and orientation of the manipulator during movement.
- Smooth interpolation between joint configurations using stepwise motion planning.

### Assignment 3: Jacobian Matrix and Inverse Kinematics

#### **Exercise 1: Jacobian Matrix**
- **Objective:** Compute the Jacobian matrix for different joint configurations.
- **Results:**
  - End-effector Jacobians calculated for precise control of the manipulator.
  - Demonstrated the relationship between joint velocities and end-effector velocities.

#### **Exercise 2 and 3: Inverse Kinematics Control**
- **Objective:** Implement inverse kinematics to control the manipulator's end-effector.
- **Steps:**
  1. Calculate the error between the current and desired end-effector poses.
  2. Use proportional control to minimize the error.
  3. Solve for joint velocities using the Jacobian pseudoinverse.
  4. Simulate the motion with joint limits.
- **Results:**
  - Successfully guided the end-effector to target positions with specified orientations.
  - Visualized the manipulator's motion in 3D, demonstrating accurate goal convergence.

#### Simulation Highlights:
- End-effector moved smoothly to target positions with controlled orientation.
- Implemented joint velocity limits to ensure safe operation.
- Plotted trajectories showing the manipulator's precision in achieving goals.

---

## How It Works

1. **Setup:**
   - Load the manipulator's geometric model.
   - Define initial and goal configurations for the manipulator.

2. **Direct Kinematics:**
   - Compute the transformation matrices for each link.
   - Plot the manipulator's pose in 3D space.

3. **Jacobian Computation:**
   - Calculate the Jacobian matrix for specified joint configurations.
   - Relate joint velocities to end-effector velocities.

4. **Inverse Kinematics:**
   - Compute the Cartesian error between the current and target poses.
   - Use proportional control and the Jacobian pseudoinverse to compute joint velocities.
   - Simulate the manipulator's motion step-by-step.

---

## Requirements

- MATLAB R2023a or later
- Robotics Toolbox for MATLAB
- Include the `panda.mat` file and the `include` folder in the project directory.

---

## Results Overview
- The manipulator's poses and trajectories are visualized in 3D plots.
- Animated simulations show smooth transitions between configurations.
- Jacobian computation enables precise control over end-effector motion.
- Inverse kinematics ensure the manipulator achieves target positions and orientations.
<img src="MCM Results/FKP.gif" alt="BOH" width="300"/>
---

## Authors
Wadoud Guelmami
