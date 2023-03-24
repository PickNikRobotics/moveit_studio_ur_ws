#!/bin/bash

# Custom entrypoint to start when running a developer container.

set -e

# Allow aliases in noninteractive shells and source common MoveIt Studio aliases
shopt -s expand_aliases
source "/common_aliases.sh"

# If the custom workspace has been compiled we will source the setup.bash file.
# Otherwise it is up to the developer to compile and source it in their session.
if [[ -f ${MOVEIT_STUDIO_CUSTOM_WS}/install/setup.bash ]]
then
    echo "Sourcing custom workspace"
    source "${MOVEIT_STUDIO_CUSTOM_WS}/install/setup.bash"
else
    echo -e "Custom workspace is not built. Please build it with: \ncolcon build\n"
fi

# Copies a DDS configuration file from the user's environment, if available.
/copy_dds_configs.sh

exec "$@"
