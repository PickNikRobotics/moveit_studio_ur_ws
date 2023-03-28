#!/bin/bash

# Custom entrypoint to start when running a release image.

set -e

source "${MOVEIT_STUDIO_CUSTOM_WS}/install/setup.bash"

# Copies a DDS configuration file from the user's environment, if available.
/copy_dds_configs.sh

exec "$@"
