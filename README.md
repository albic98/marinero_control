
# MARINERO Control

This package contains odometry, joystick control, and autonomy logic for the **MARINERO** robot.  
It integrates with the MPPI navigator, path planner, YOLO object detection, and provides markers for localization in RViz and Gazebo.  

---

## Requirements  

- **ROS 2 Humble** (or compatible ROS 2 distribution)  
- Packages:  
  - `ros2_control`  
  - `twist_mux`  
  - `joy`  
  - `teleop_twist_joy`
  - `nav2_mppi_controller` (optional, for MPPI-based navigation)  
  - `nav2_planner` (optional, for path planning) 

---

## Installation

Clone the repository in your ROS2 workspace.

```
  cd workspace_folder/src
  git clone https://github.com/albic98/marinero_control.git
```

Then build the workspace in your `workspace_folder`.

```
  source /opt/ros/<distro>/setup.bash
  colcon build --symlink-install
  source install/setup.bash
```

---

## Usage/Examples

#### Visualize location markers in RViZ and Gazebo:
```
  ros2 run marinero_control marinero_tracker
  ros2 run marinero_control gazebo_marker
```

#### Start the robot odometry:
```
  ros2 run marinero_control marinero_odometry_new_model
```

#### Enable joystick control:
```
  ros2 run marinero_control marinero_teleop
```

#### Start robot control logic with autonomy:
```
  ros2 run marinero_control marinero_control_navigator
```

#### Run YOLO network for object detection:
```
  ros2 run marinero_control marinero_yolo
```

## Deprecated Scripts (Not Used Anymore)

These scripts belong to older versions of the MARINERO system and are no longer used in the current workflow.

- `docking_station.py` - Early script for sending the robot to its HOME position

- `marinero_camera.py` - Previously used for manual PTZ camera control via terminal

- `marinero_camera_w_joy.py` - Old joystick-based PTZ camera controller (fully unused)

- `marinero_camera_w_keyboard.py` - Old keyboard-based PTZ camera controller (fully unused)

- `marinero_odometry_old_model.py` - Odometry implementation for the previous robot model.
  - Can still be used if the older robot description is restored. 
  - The corresponding old robot model is located in: `marinero_simulations/robot_description/old_model_robot_description`

---

## Support

For support, go to https://github.com/CRTA-Lab/marinero_stack.

---

## Connection to the MARINERO simulations repository

The following nodes from this package:
  - `marinero_control_navigator`
  - `marinero_odometry_new_model`
  - `marinero_yolo`
  - `gazebo_marker`
  - `marinero_tracker`
  - `marinero_teleop` 

  are automatically launched via
`_4wis4wid_drive_joystick.launch.py` and `gazebo_simulation.launch.py` in the [marinero_simulations](https://github.com/albic98/marinero_simulations) package. 
This means they are included automatically when running the full simulation.