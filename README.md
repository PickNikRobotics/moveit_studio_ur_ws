# MoveIt Studio Workspace

This is a sample user workspace for running MoveIt Studio with a generic Universal Robots arm.
For more information, refer to the [MoveIt Studio Documentation](https://docs.picknik.ai/).

Instructions for building your own MoveIt Studio configuration can be found [here.](https://docs.picknik.ai/en/stable/concepts/config_package/config_package.html)

---
## MoveIt Studio Configuration

This package follows the recommended layout of a MoveIt Studio configuration package.
Any configuration package or custom Behavior implementation can be included in the `src/` directory.
Packages in `src/` will be compiled and sourced by MoveIt Studio at first launch.

### Site Config vs Base Config

MoveIt Studio supports two types of site configuration packages, a base config and site config.

[Base configs](src/picknik_ur_base_config/README.md) are used to configure all of the system components that remain unchanged when deploying the robot to a new location.

[Site configs](src/picknik_ur_site_config/README.md) are used to specify additional features or constraints to be added to the robot such as additional cameras, sensors, and actuators as well as obstacles (like walls), joint speed and torque limits, network settings, waypoint locations, camera calibration, and more.

### Writing Your Own Configuration

At PickNik we use a UR5e base config package which all of our UR5e robot site configurations are based on. For each different set-up of a UR5e robot or simulation environment, we have a site configuration package. This allows us to have one UR5e in a highly cluttered environment, another UR5e with increased velocity and torque allowances, and an entirely simulated UR5e in the world of our choosing. All of which are able to make use of MoveIt Studio's full suite of motion planning and manipulation capabilities.

For ease of use, this package is provided as a base config. We have taken our UR5e base configuration, incorporated the additions we use in our UR5e simulation site configuration and released it as a standalone base configuration package. This package provides a configuration that can be used with MoveIt Studio and Gazebo to simulate and explore the capabilities of MoveIt Studio controlling a UR5e.

Though you can modify this package to build a different configuration of your choosing, it is recommended to follow our [Configuring Studio for a Specific Robot](https://docs.picknik.ai/en/stable/concepts/config_package/config_package.html) tutorial and create your own site configuration, using this package as the base config.

---
## Beyond site configuration

If you're having fun [building awesome new Objectives](https://docs.picknik.ai/en/stable/tutorials/use_objectives_in_ui/use_objectives_in_ui.html) in MoveIt Studio, and would like to take your experience a step further, check out our tutorials on [creating custom Behaviors](https://docs.picknik.ai/en/stable/tutorials/create_behavior/create_behavior.html).
Reach out to your MoveIt Studio license provider if you want to try out and experience your custom Objectives on one of PickNik's robots.

---
# Launching a Mock Hardware Robot

**This repository assumes you have followed the [installation instructions online](https://docs.picknik.ai/en/stable/getting_started/software_installation/software_installation.html).**

To start MoveIt Studio with the default configuration with a UR5e, you must specify this repository as the target `USER_WS` in the provided [.env file](.env).
By default, `STUDIO_HOST_USER_WORKSPACE` points to `$HOME/moveit_studio_ws`.
If you have cloned this repository to a different location, then update the line to point to this workspace.

Once that is done, from the root of the workspace run:

`docker compose up`

Wait a moment for the application to start, then open the Chrome browser and navigate to [http://localhost](http://localhost).

You should be up and running with the default UR5e configuration!
