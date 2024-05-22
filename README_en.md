
# PybulletRobotics

【[Japanese version](https://github.com/akinami3/PybulletRobotics/)】

This repository introduces basic usage of pybullet in robotics and how to implement various algorithms related to robotics using pybullet.

# Table of Contents
- [PybulletRobotics](#pybulletrobotics)
- [Table of Contents](#table-of-contents)
- [Installation](#installation)
- [Basic Usage of Pybullet in Robotics](#basic-usage-of-pybullet-in-robotics)
- [Mobile Robots](#mobile-robots)
  - [Basics](#basics)
    - [Basic Control of Mobile Robots](#basic-control-of-mobile-robots)
    - [Using Sensors in Mobile Robots](#using-sensors-in-mobile-robots)
  - [Line Tracing with Mobile Robots](#line-tracing-with-mobile-robots)
  - [Other Content to be Added](#other-content-to-be-added)
- [Robot Arms](#robot-arms)
  - [Basics](#basics-1)
    - [Basic Control of Robot Arms](#basic-control-of-robot-arms)
    - [Using Sensors in Robot Arms](#using-sensors-in-robot-arms)
    - [Collision Detection](#collision-detection)
  - [Kinematics](#kinematics)
    - [Forward Kinematics Using Trigonometry (To be added)](#forward-kinematics-using-trigonometry-to-be-added)
    - [Forward Kinematics Using Homogeneous Transformation Matrices (To be added)](#forward-kinematics-using-homogeneous-transformation-matrices-to-be-added)
    - [Inverse Kinematics Using Analytical Methods (To be added)](#inverse-kinematics-using-analytical-methods-to-be-added)
    - [Inverse Kinematics Using Jacobian Matrices with Numerical Methods (To be added)](#inverse-kinematics-using-jacobian-matrices-with-numerical-methods-to-be-added)
  - [Other Content to be Added](#other-content-to-be-added-1)

<!-- Major robots not covered in this repository (as of now)
- Drones
- Humanoid Robots
- Soft Robotics
- Robot Hands
- Quadruped Robots
- Spider-like Robots
- Snake-like Robots -->

# Installation
The steps to install the libraries necessary for performing robotics simulations with pybullet are as follows.

Environment: Ubuntu (WSL is also acceptable)

```bash
sudo apt update
```

```bash
sudo apt install python3-pip
```

```bash
pip3 install pybullet
```

```bash
pip3 install numpy
```

```bash
pip3 install matplotlib
```

```bash
pip3 install opencv-contrib-python
```

```bash
pip install scipy
```

# Basic Usage of Pybullet in Robotics
Below are the basic usage methods of pybullet in robotics.
- Creating an environment
- Loading a robot from a urdf file
- Running a simulation

For details, refer to [pybullet_basic.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/PybulletBasic/pybullet_basic.ipynb).

# Mobile Robots

## Basics

### Basic Control of Mobile Robots
Here is a simple code to move a two-wheel mobile robot.

<img src="./images/mobile_robot_basic.gif" width="65%">

For details, refer to [mobile_robot_basic.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/MobileRobot/mobile_robot_basic.ipynb).

### Using Sensors in Mobile Robots
Introduction to using the following sensors on a two-wheel mobile robot:
- Lidar
- Camera
- IMU

<img src="./images/mobile_robot_sensor.png" width="65%">

For details, refer to [mobile_robot_sensor.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/MobileRobot/mobile_robot_sensor.ipynb).

## Line Tracing with Mobile Robots
Introduction to line tracing with a two-wheel mobile robot.

<img src="./images/mobile_robot_line_trace.gif" width="65%">

For details, refer to [mobile_robot_line_trace.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/MobileRobot/mobile_robot_line_trace.ipynb).

## Other Content to be Added
- SLAM
- Path Planning
- Motion Planning
- Obstacle Avoidance

# Robot Arms

## Basics

### Basic Control of Robot Arms
Introduction to basic control of a 2-axis robot arm.

<img src="./images/robot_arm_basic_position_control.gif" width="55%">

For details, refer to [robot_arm_basic.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_basic.ipynb).

### Using Sensors in Robot Arms
Introduction to using the following sensors on a 2-axis robot arm:
- Tip Camera
- Tip Force Sensor

<img src="./images/2d_robot_arm_sensor.png" width="80%">

<br>

**Tip Camera**

<img src="./images/robot_arm_tip_camera.gif" width="55%">

<br>

**Tip Force Sensor**

<img src="./images/robot_arm_tip_force_sensor.gif" width="55%">

For details, refer to [robot_arm_sensor.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_sensor.ipynb).

### Collision Detection
Introduction to collision detection between a robot arm and an object.

<img src="./images/robot_arm_collision_check.gif" width="55%">

For details, refer to [robot_arm_collision_check.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_collision_check.ipynb).

## Kinematics

### Forward Kinematics Using Trigonometry (To be added)
To be added
<!-- Introduction to forward kinematics of robot arms using trigonometry.

For details, refer to [robot_arm_trigonometric_forward_kinematics.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_trigonometric_forward_kinematics.ipynb). -->

### Forward Kinematics Using Homogeneous Transformation Matrices (To be added)
To be added

<!-- Introduction to forward kinematics of robot arms using homogeneous transformation matrices.

For details, refer to [robot_arm_homogeneous_matrix_forward_kinematics.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_homogeneous_matrix_forward_kinematics.ipynb). -->

### Inverse Kinematics Using Analytical Methods (To be added)
To be added

<!-- Introduction to inverse kinematics of robot arms using analytical methods.

For details, refer to [robot_arm_analytical_inverse_kinematics.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_analytical_inverse_kinematics.ipynb). -->

### Inverse Kinematics Using Jacobian Matrices with Numerical Methods (To be added)
To be added

<!-- Introduction to inverse kinematics of robot arms using Jacobian matrices with numerical methods.

For details, refer to [robot_arm_jacobian_inverse_kinematics.ipynb](https://github.com/akinami3/PybulletRobotics/blob/main/RobotArm/robot_arm_jacobian_inverse_kinematics.ipynb). -->

## Other Content to be Added
- PID Control
- Motion Planning in Task Space
- Motion Planning Using Configuration Space
- Force Control
- Hybrid Position and Force Control
- Impedance Control
- Visual Servo
