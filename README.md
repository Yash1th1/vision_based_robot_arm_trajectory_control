# Vision-Based Robot Arm Trajectory Control

## Overview

This project involves programming a KUKA LBR iiwa 7 R800 robotic arm to position a rectangular shape on top of a rectangular target in the robot's workspace. The target's exact location and orientation are determined using an Aruco marker, captured by a camera attached to the robot flange. The trajectory is planned in joint space to move the robot arm from its starting configuration to the target while ensuring compliance with joint constraints and achieving the motion in exactly 10 seconds.

## Features

- **Forward Kinematics**: Calculate the transformation matrix from the base of the robot to the end-effector.
- **Inverse Kinematics**: Determine the joint angles required to achieve the desired end-effector pose.
- **Trajectory Planning**: Generate a smooth trajectory for the robot arm to follow from its initial position to the target.
- **Constraint Verification**: Ensure that the robot's joint angles and velocities remain within specified limits throughout the motion.

## Repository Contents

- `main.m`: Main script for executing the entire project workflow, including kinematics, trajectory planning, and verification.
- `functions`: Directory containing helper functions for kinematics, Jacobian calculations, and more.
  - `forward_kinematics.m`: Function to compute the forward kinematics using DH parameters.
  - `DH_transform.m`: Function to compute individual transformation matrices based on DH parameters.
  - Additional helper functions for Jacobian calculation, inverse kinematics, and trajectory generation.
- `Tekumudi_Yashwanth.txt`: Output file containing the joint angles at each timestep of the trajectory.
- `Tekumudi_Yashwanth_Velocity.txt`: Output file containing the joint velocities at each timestep for verification purposes.

## Usage

1. **Initialization**: Set the initial joint configuration (`qc`) and DH parameters.
2. **Forward Kinematics**: Compute the transformation matrix for the end-effector.
3. **Desired Pose Calculation**: Use the Aruco marker to determine the target's pose and compute the desired end-effector pose.
4. **Inverse Kinematics**: Calculate the joint angles required to achieve the desired end-effector pose.
5. **Trajectory Planning**: Generate a smooth trajectory for the robot to follow from its initial position to the target position.
6. **Verification**: Check the generated trajectory to ensure joint constraints are met.

## Notes

- Ensure all necessary MATLAB toolboxes are installed for functions like `eul2rotm` and `rotm2eul`.
- The trajectory is planned to last exactly 10 seconds, and joint limits are checked to ensure safe operation.

