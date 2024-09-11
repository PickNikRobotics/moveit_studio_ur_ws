import xacro

from pathlib import Path

from moveit_studio_utils_py.create_urdf import process_mappings_dict
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)

system_config_parser = SystemConfigParser()
hardware_config = system_config_parser.get_hardware_config()

params = hardware_config.robot_description
mappings_params = process_mappings_dict(params.urdf_params)

# IMPORTANT: You must update this path when using it to debug other configurations!
robot_description_path = Path("~/user_ws/src/arm_on_rail_sim/xacro_generated_urdf_for_testing.urdf").expanduser()

robot_description_config = xacro.process_file(
    robot_description_path,
    mappings=mappings_params,
)

print(robot_description_config.toxml())
