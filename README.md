# MoveIt Studio Workspace

This is a sample user workspace for running MoveIt Studio with a generic Universal Robots arm.
For more information, refer to the [MoveIt Studio Documentation](https://docs.picknik.ai/).

Instructions for building your own MoveIt Studio configuration can be found [here.](https://docs.picknik.ai/en/stable/concepts/config_package/config_package.html)

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
For more information refer to the [online documentation](https://docs.picknik.ai/en/stable/concepts/config_package/config_package.html).

## Beyond site configurations

A custom site configuration is a great jumping off point for starting development with MoveIt Studio.

We recommend reviewing the [online documentation](https://docs.picknik.ai/en/stable/) for more information on building Objectives, implementing custom Behaviors, and integrating peripherals into the MoveIt Studio application.

# Launching a Mock Hardware Robot

**This repository assumes you have followed the [installation instructions online](https://docs.picknik.ai/en/stable/getting_started/software_installation/software_installation.html).**
If that is the case, follow [these instructions](https://docs.picknik.ai/en/stable/getting_started/configuring_moveit_studio/configuring_moveit_studio.html) for updating and running the configuration provided in this repository.

Otherwise, it is left to the user to ensure that the prerequites from the installation process have been met.

To manually launch MoveIt Studio with the default configuration provided by this workspace, specify this repository as the target `STUDIO_HOST_USER_WORKSPACE` in your MoveIt Studio configuration.
By default, `STUDIO_HOST_USER_WORKSPACE` points to `$HOME/moveit_studio_ws`.
If you have cloned this repository to a different location, then update the line to point to this workspace.

Once that is done, from the root of the workspace run:

`docker compose up`

Wait a moment for the application to start, then open the Chrome browser and navigate to ``http://localhost``.

You should be up and running with the default UR5e configuration!
