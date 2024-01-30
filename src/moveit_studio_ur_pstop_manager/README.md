# Protective Stop Recovery

The `ProtectiveStopManager` class provides a service that allows the user to recover when the robot enters a protective stop state and to resume normal execution.
At the moment, this feature is implemented as a service that calls other services, namely [`unlock_protective_stop`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/a6e209d393b35b3c67015e022ce4a4eff238a111/ur_robot_driver/src/dashboard_client_ros.cpp#L95-L96) and [`resend_robot_program`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/a6e209d393b35b3c67015e022ce4a4eff238a111/ur_controllers/src/gpio_controller.cpp#L271-L273).
In the future, it would be preferable to implement this feature differently, so that one ROS service does not have to call other services.

The [`resend_robot_program` callback](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/a6e209d393b35b3c67015e022ce4a4eff238a111/ur_controllers/src/gpio_controller.cpp#L363) must be implemented by a controller.
So, one option is to re-implement the [ROS Dashboard Client](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/src/dashboard_client_ros.cpp) as a controller rather than a node, and to use this new controller to advertise the `resend_robot_program` service (instead of the [`GPIOController`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_controllers/src/gpio_controller.cpp)).
This controller could also create the `recover_from_protective_stop` service (currently created by the ProtectiveStopManager).  `recover_from_protective_stop` could then directly call the functions that implement `unlock_protective_stop` and `resend_robot_program`, rather than calling those services.
