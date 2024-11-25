# WARNING This package is deprecated an no longer maintined

A more complete reference workspace is provided here:

[moveit_pro_example_ws](https://github.com/PickNikRobotics/moveit_pro_example_ws)

An empty workspace can be found here:

[moveit_pro_empty_ws](https://github.com/PickNikRobotics/moveit_pro_empty_ws)

The [moveit_pro_ur_configs](https://github.com/PickNikRobotics/moveit_pro_ur_configs) folder had been synced with the upstream repository using subtrees.
To pull the latest changes, use:
```bash
git subtree pull --prefix src/moveit_pro_ur_configs https://github.com/PickNikRobotics/moveit_pro_ur_configs main --squash
```
Similarly, the [moveit_pro_mobile_manipulation](https://github.com/PickNikRobotics/moveit_pro_mobile_manipulation) folder can be synced using:
```bash
git subtree pull --prefix src/moveit_pro_mobile_manipulation https://github.com/PickNikRobotics/moveit_pro_mobile_manipulation main --squash
```

# MoveIt Pro Workspace for Universal Robots Arms

This is a sample user workspace for running MoveIt Pro with a generic Universal Robots (UR) arm.
For more information, refer to the [MoveIt Pro Documentation](https://docs.picknik.ai/).

Instructions for building your own MoveIt Pro configuration can be found [in the getting started guides](https://docs.picknik.ai/docs/getting_started/setup_tutorials/software_installation/).

MoveIt Pro can be used with real robots and full simulators such as Gazebo and NVIDIA Isaac Sim.
For testing purposes, you can also use the [ROS 2 Control Mock Components](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html), which is what this repo is configured to use.

This workspace offers a reasonable starting point for those users looking to develop with MoveIt Pro using custom base and site configurations.
For more information refer to the [online documentation](https://docs.picknik.ai).

## Universal Robots Configuration Packages

This workspace contains several MoveIt Pro configuration packages for UR arms. 

Some of the packages inherit from a base configuration:

```mermaid
graph TB
Base[picknik_ur_base_config] --> Rail[arm_on_rail_sim]
Base --> Mock[picknik_ur_mock_hw_config]
Base --> Site[picknik_ur_site_config (hardware-ready)]
```

* `picknik_ur_base_config` contains common configuration for all UR arms, real or simulated.
* `picknik_ur_mock_hw_config` provides overrides for a machine tending application simulated using mock components (no physics).
* `picknik_ur_site_config` extends the base configuration with capabilities for robots with physics and perception.
* `picknik_ur_multi_arm_config` is an example configuration for using multiple arms.
* `arm_on_rail_sim` is an example configuration of a UR5e on a linear rail.
