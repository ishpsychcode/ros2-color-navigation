# ROS2 Color-Based Navigation using OpenCV

## Overview

This project implements autonomous color-based navigation for TurtleBot3 using ROS2 Humble, OpenCV, and Gazebo simulation.

The robot:

- Detects a green object using OpenCV
- Finds the contour and centroid of the object
- Computes horizontal error from image center
- Uses a proportional (P) controller for alignment
- Moves toward the object when aligned
- Stops when the object is lost or obstacle is near

---

# Features

- ROS2 Humble compatible
- TurtleBot3 Gazebo simulation
- OpenCV color detection
- Contour extraction
- Centroid computation
- P-controller based steering
- Autonomous object tracking and navigation

---

# Technologies Used

- ROS2 Humble
- TurtleBot3
- Gazebo
- Python
- OpenCV
- cv_bridge
- NumPy

---

# Project Structure

```bash
ros2_ws/
│
├── src/
│   └── color_tracker/
│       ├── color_tracker/
│       │   ├── __init__.py
│       │   └── image_subscriber.py
│       │
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
└── README.md

---

# Problem Statement

Implement color-based navigation using ROS2 and OpenCV.

Tasks completed:

Setup TurtleBot3 Gazebo simulation
Subscribe to camera image topic
Detect green object using OpenCV
Find contour and centroid
Compute horizontal error
Implement proportional controller for robot alignment
Move robot toward object when aligned
Discuss recovery and stopping strategies

---

# Installation and Setup

## 1. Install ROS2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

---

## 2. Install TurtleBot3 Packages

```bash
sudo apt install ros-humble-turtlebot3* -y
sudo apt install ros-humble-turtlebot3-gazebo -y
```

---

## 3. Install OpenCV Dependencies

```bash
sudo apt install ros-humble-cv-bridge python3-opencv -y
```

---

## 4. Set TurtleBot3 Model

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

---

# Running the Simulation

## Launch Gazebo

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## Spawn Green Sphere

```bash
ros2 run gazebo_ros spawn_entity.py \
-file ~/models/green_sphere/model.sdf \
-entity green_sphere \
-x 1.5 \
-y 0.0 \
-z 0.5
```

---

## Build Workspace

```bash
cd ~/ros2_ws
colcon build
```

---

## Source Workspace

```bash
source install/setup.bash
```

---

## Run the Node

```bash
ros2 run color_tracker image_subscriber
```

---

# Methodology

## Step 1 — Camera Subscription

A ROS2 node subscribes to:

```text
/camera/image_raw
```

using:

```python
self.subscription = self.create_subscription(
    Image,
    '/camera/image_raw',
    self.image_callback,
    10
)
```

---

## Step 2 — OpenCV Image Processing

The ROS image is converted into OpenCV format using CvBridge.

```python
frame = self.bridge.imgmsg_to_cv2(
    msg,
    desired_encoding='bgr8'
)
```

---

## Step 3 — HSV Color Thresholding

The image is converted into HSV color space and thresholded for green color detection.

```python
lower_green = np.array([40, 50, 50])
upper_green = np.array([80, 255, 255])
```

---

## Step 4 — Contour Detection

Contours are extracted using:

```python
cv2.findContours()
```

The largest contour is selected as the target object.

---

## Step 5 — Centroid Calculation

The centroid of the detected object is computed using image moments.

```python
cx = int(M["m10"] / M["m00"])
cy = int(M["m01"] / M["m00"])
```

---

## Step 6 — Horizontal Error Calculation

Horizontal error is calculated as:

```python
error_x = cx - image_center_x
```

This determines whether the object is positioned left or right of the image center.

---

## Step 7 — Proportional Controller

The proportional controller equation used is:

```python
angular_z = -kp * error_x
```

The robot rotates proportionally toward the object.

---

## Step 8 — Forward Navigation

If the object is sufficiently aligned:

```python
if abs(error_x) < threshold:
```

the robot moves forward toward the object.

---

# Working Principle

- TurtleBot3 camera captures image frames
- ROS2 node subscribes to camera topic
- OpenCV processes image frames
- Green object is detected using HSV thresholding
- Contours are extracted from thresholded image
- Largest contour is selected
- Object centroid is computed
- Horizontal alignment error is calculated
- P-controller rotates the robot toward the object
- Robot moves forward when alignment condition is satisfied

---

# P Controller Equation

```text
angular_z = -Kp × error_x
```

Where:

- `Kp` = proportional gain
- `error_x` = horizontal offset from image center

---

# Recovery Behavior

If the object goes out of frame:

- the robot stops forward motion
- rotates in the last known direction
- searches until the object reappears
- resumes normal tracking after detection

---

# Obstacle Distance Estimation

The TurtleBot3 LiDAR topic:

```text
/scan
```

can be used to:

- estimate obstacle distance
- determine proximity to object
- stop the robot safely near obstacles

---

# Results

The robot successfully:

- detects a green object
- tracks object position
- computes centroid and alignment error
- rotates toward the object
- autonomously moves toward the object
- performs color-based navigation in Gazebo simulation

---

# Future Improvements

- PID controller implementation
- Multi-object tracking
- Dynamic obstacle avoidance
- Kalman filter based tracking
- Depth estimation using stereo vision
- Integration with SLAM and autonomous navigation stack

---

# Author

Name: Ishita Yadav
