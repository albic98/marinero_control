
# Project title

Marinero digital twin description files, controllers and localization logic
## Getting started

After cloning extract files `robot_description/meshes.zip`.

```
  unzip robot_description/meshes.zip
```

If you do not have `unzip` installed use

```
  sudo apt-get install unzip
```
to install it.

## Support

One `MARINERO_chassis.stl` and one `MARINERO_chassis.dae` file is missing from the repository because they are too large for Github. To access, them write an e-mail to the email written below.

For support, email albert.androsic@fsb.unizg.hr.


## Usage/Examples

Start the robot state publisher for MARINERO:

```
  ros2 launch marinero_simulations rsp.launch.py
```

Initiate the entire simulation using this command (4WIS4WID controller) with `ros2_control.xacro`:
```
  ros2 launch marinero_simulations gazebo_simulation.launch.py
```

Launch command for controllers if they do not start with previous command:
```
  ros2 launch marinero_simulations controllers.launch.py
```

Initiate the entire simulation using this command (2 differential drive controller) with `gazebo_control.xacro`:
```
  ros2 launch marinero_simulations gazebo_simulation.launch.py use_ros2_control:=false use_4wis4wid:=false
```

