# ROS 2 Package for LeRobot SO-ARM101

LeRobot SO-ARM101 integrated into ROS 2 Jazzy.

## Features

- ✅ ROS 2 Jazzy compatibility
- ✅ Rviz visualization
- ✅ Gazebo Harmonic simulation
- ✅ ROS 2 Control integration
- ✅ MoveIt 2 motion planning
- 📝 **TODO:** ROS 2 control interface for the real HW
---
## Installation

Clone this repository and install dependencies using [rosdep](https://docs.ros.org/en/ros2_packages/rosdep.html):


### Clone the repository
`git clone https://github.com/Pavankv92/lerobot_ws.git`

`cd lerobot_ws`

### Install ROS 2 dependencies
`rosdep update`

`rosdep install --from-paths src --ignore-src -r -y`

### Build
`colcon build`

---
## Rviz

**Summary:** Visualising LeRobot SO101 in Rviz

**Command:**  
`ros2 launch lerobot_description so101_display.launch.py`

**Video:**  
<!-- Add your video link here -->
https://github.com/user-attachments/assets/98f0a867-46c5-4661-8308-5de9e60a960b

---

## Gazebo and ROS 2 Control

**Summary:** Gazebo and ROS 2 Control: Control the gripper

**Commands:**  
`ros2 launch lerobot_description so101_gazebo.launch.py`  
`ros2 launch lerobot_controller so101_controller.launch.py`

**Video:**  
<!-- Add your video link here -->


https://github.com/user-attachments/assets/7d82b15c-8276-43b1-9b73-00b3567a5cf7


---

## Gazebo, ROS 2 Control and MoveIt

**Summary:** Gazebo, ROS 2 Control and MoveIt 2: MoveIt planner for the arm and gripper

**Commands:**  
`ros2 launch lerobot_description so101_gazebo.launch.py`  
`ros2 launch lerobot_controller so101_controller.launch.py`  
`ros2 launch lerobot_moveit so101_moveit.launch.py`

**Settings:**
- select "ompl" planning library for "arm" and "gripper" groups 

**Video: Arm**  
<!-- Add your video link here -->


https://github.com/user-attachments/assets/f95e9fd7-272a-46a1-8b34-0cb6c3f36da8

**Video: Gripper**  
<!-- Add your video link here -->

https://github.com/user-attachments/assets/5511c329-faad-4020-9527-4034f54a027a

---

## License

This project is based on [RobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) and adheres to their license.







