#!/bin/bash

# Custom entrypoint to start when running an image that overlays MoveIt Studio

set +e

# Set up the ROS workspaces at startup.
# shellcheck source=bin/docker/utils/setup_workspaces.sh
source /moveit_studio_utils/setup_workspaces.sh

# Set the permissions for your non-root user at startup.
# shellcheck source=bin/docker/utils/change_permissions.sh
source /moveit_studio_utils/change_permissions.sh

# Set the DDS configuration for ROS 2 inter-process communication.
# shellcheck source=bin/docker/utils/copy_dds_configs.sh
source /moveit_studio_utils/copy_dds_configs.sh

exec "$@"
