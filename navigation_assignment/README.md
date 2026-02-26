# How to launch our project

Follow these simple steps to compile and launch our ROS 2 project.

---
Before using ROS 2, make sure to source the environment:

## 1. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
````

---
Navigate to your workspace, build it, and source the setup file:

## 2. Build the Workspace

```bash
cd ~/ws_17_assignments
colcon build
source install/setup.bash
```

---
Run the launch file to start the nodes:

## 3.1 Launch the Package

```bash
ros2 launch group17_assignment_1 assignment_launch.py
```
---
For the extra part, run the following launch file:

## 3.2 Launch the Extra Package

```bash
ros2 launch group17_assignment_1 extra_launch.py
```
---
