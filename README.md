# MoveIt Studio Workspace

This is a sample user workspace for running MoveIt Studio with a generic Universal Robots (UR) arm.
For more information, refer to the [MoveIt Studio Documentation](https://docs.picknik.ai/).

Instructions for building your own MoveIt Studio configuration can be found [in the getting started guides.](https://docs.picknik.ai/en/stable/getting_started/getting_started.html)

MoveIt Studio can be used with real robots and full simulators such as Gazebo and NVIDIA Isaac Sim.
For testing purposes, you can also use the [ROS 2 Control Mock Components](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html), which is what this repo is configured to use.

---
## MoveIt Studio Configuration

This package follows the recommended layout of a MoveIt Studio configuration package.
Any configuration package or custom Behavior implementation can be included in the `src/` directory.
Packages in `src/` will be compiled and sourced by MoveIt Studio at first launch.

### Concepts

MoveIt Studio supports two types of site configuration packages, a base config and site config.

[Base configs](src/picknik_ur_base_config/README.md) are used to configure all of the system components that remain unchanged when deploying the robot to a new location.

[Site configs](src/picknik_ur_site_config/README.md) are used to override any parameters of the base configuration, or add additional features or constraints for a particular installation.

This workspace offers a reasonable starting point for those users looking to develop with MoveIt Studio using custom base and site configurations.
For more information refer to the [online documentation](https://docs.picknik.ai/en/stable/).
