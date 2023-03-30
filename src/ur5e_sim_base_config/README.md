# picknik_ur_base_config

A MoveIt Studio base configuration for PickNik's UR5e simulation, for detailed documentation see: [MoveIt Studio Documentation](https://docs.picknik.ai/)

# Structure of a MoveIt Studio configuration package

This package follows the common layout of a MoveIt Studio configuration package. Instructions for building your own site configuration can be found [here.](https://docs.picknik.ai/en/stable/concepts/config_package/config_package.html)

# Site config vs Base config

MoveIt Studio supports two types of site configuration packages, a base config and site config. Base configs are used to configure all of the system components that remain unchanged when deploying the robot to a new location. Site configs are used to specify additional features or constraints to be added to the robot such as additional cameras, sensors, and actuators as well as obstacles (like walls), joint speed and torque limits, network settings, waypoint locations, cameral calibration, and more. At PickNik we use a UR5e base config package on which all of our UR5e robot site configurations are based on. For each different set-up of a UR5e robot or simulation environment, we have a site configuration package. This allows us to have one UR5e in a highly cluttered environment, another UR5e with increased velocity and torque allowances, and an entirely simulated UR5e in the world of our choosing. All of which are able to make use of MoveIt Studio's full suite of motion planning and manipulation capabilities.

For ease of use, this package is provided as a base config. We have taken our UR5e base configuration, incorporated the additions we use in our UR5e simulation site configuration and released it as a standalone base configuration package. This package provides a configuration that can be used with MoveIt Studio and Gazebo to simulate and explore the capabilities of MoveIt Studio controlling a UR5e. Though you can modify this package to build a different configuration of your choosing, it is recommended to follow our [Configuring Studio for a Specific Robot](https://docs.picknik.ai/en/stable/concepts/config_package/config_package.html) tutorial and create your own site configuration, using this package as the base config.

# Beyond site configuration

If you're having fun [building awesome new objectives](https://docs.picknik.ai/en/stable/tutorials/use_objectives_in_ui/use_objectives_in_ui.html) in MoveIt Studio, and would like to take your experience a step further, check out our tutorials on [Creating custom behaviors](https://docs.picknik.ai/en/stable/tutorials/create_behavior/create_behavior.html). Reach out to your studio license provider if you want to try out and experience your custom objectives on one of PickNik's robots.
