# ROS2 Mobile Navigation and Manipulation Portfolio

## Overview
This portfolio integrates two university assignments focused on **mobile robot navigation** and **manipulator motion planning** using **ROS2** and **Gazebo**.  
The projects simulate autonomous navigation, corridor traversal, table detection, and pick-and-place tasks using a UR5 manipulator.

The goal of this portfolio is to showcase both **software design** and **ROS2 system implementation**, including perception, motion planning, and high-level orchestration.

---

## System Architecture
- **ROS2 Nodes**: Modular nodes handling navigation, corridor traversal, table detection, and manipulation.  
- **Nav2**: Autonomous navigation stack for the mobile robot.  
- **MoveIt**: Motion planning and execution for the manipulator.  
- **Gazebo Simulation**: Simulated robot environment including world files and objects.  
- **TF2**: Frame transformations for sensor and robot integration.

For detailed system architecture, diagrams, and node descriptions, see the [assignment reports](#reports).

---

## My Contributions
- Developed and integrated **AprilTag nodes** for object detection and pose estimation, including processing detections and publishing navigation goals.
- Implemented **Corridor Nodes** for reliable corridor detection and following, including PD-based wall-centering control and integration with the manager node.
- Contributed to the **Manager Node** of Assignment 2, orchestrating the pick-and-place pipeline and coordinating perception, planning, and actuation.
- Designed and implemented **Recovery Strategies** to handle simulation and motion planning failures, ensuring robust task execution.
- Performed **Kinematics and TF analysis** for both mobile navigation and manipulator tasks, ensuring correct frame transformations and precise motion execution.
- Main files authored by me:
  - `navigation_assignment/src/apriltag_listener_node.py`
  - `navigation_assignment/src/apriltag_processor_node.py`
  - `navigation_assignment/src/corridor_detector_node.py`
  - `navigation_assignment/src/corridor_follower_node.py`
  - `manipulation_assignment/src/manager_node.py`

---

## How to Run
For detailed launch instructions and setup, see the **README** in each assignment folder:

- [Navigation Assignment Instructions](navigation_assignment/README.md)  
- [Manipulation Assignment Instructions](manipulation_assignment/README.md)

---

## Demo
Simulation demonstrations are provided in the `media/` folder:

- `media/nav_demo.gif` – Mobile navigation and corridor traversal  
- `media/manipulation_demo.gif` – Pick-and-place with UR5 manipulator  


---

## Original Repositories
- Navigation Assignment: [ws_17_assignments](https://github.com/mattreturn1/ws_17_assignments)  
- Manipulation Assignment: [assignment_2](https://github.com/mattreturn1/assignment_2)  

---

## Reports
Detailed documentation for each assignment is included in the `docs/` folder:  

- [Assignment 1 Report](docs/assignment1_report.pdf)  
- [Assignment 2 Report](docs/assignment2_report.pdf)

These reports include system architecture diagrams, node-level explanations, and high-level algorithms.

---

## Acknowledgments
This work was done as part of a university course project.  
Original code contributions were shared with the group, and all technical content was verified by the team.  
AI assistance (ChatGPT) was used only for structuring the LaTeX reports.
