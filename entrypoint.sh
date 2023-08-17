#!/bin/bash

# Custom entrypoint to start when running an image that overlays MoveIt Studio

set +e

# Source the MoveIt Studio binaries and your workspace
source "/opt/overlay_ws/install/setup.bash"
if [ -f "${USER_OVERLAY_WS}/install/local_setup.bash" ]; then
  source "${USER_OVERLAY_WS}/install/local_setup.bash"
fi

# Set the location for custom Behavior package generation
export STUDIO_GENERATE_PACKAGE_OUTPUT_PATH="${USER_OVERLAY_WS}/src"

# Set the permissions for your non-root user at startup.
# shellcheck source=bin/docker/utils/change_permissions.sh
source /moveit_studio_utils/change_permissions.sh

# Set the DDS configuration for ROS 2 inter-process communication.
# shellcheck source=bin/docker/utils/copy_dds_configs.sh
source /moveit_studio_utils/copy_dds_configs.sh

exec "$@"
