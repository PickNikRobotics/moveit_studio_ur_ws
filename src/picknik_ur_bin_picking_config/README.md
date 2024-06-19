# picknik_ur_bin_picking_config

This is a MoveIt Pro configuration for a pick-and-place workcell which uses a Universal Robot arm and Robotiq E-Pick gripper.

The "Looping Pick and Place Object SAM" objective demonstrates bin picking with an external 2D segmentation server to label object instances. This objective was developed and tested using the perception pipeline provided in the [PickNikRobotics/moveit_studio_segment_anything](https://github.com/PickNikRobotics/moveit_studio_segment_anything/) repo. Follow the installation and setup instructions in that repo before running the objective.

## Visualization

To view this robots URDF in Rviz:
``` bash
ros2 launch picknik_ur_bin_picking_config view_robot.launch.py
```
