#!/bin/bash

# Custom entrypoint to start when running a developer container.

set -e

# Allow aliases in noninteractive shells and source common MoveIt Studio aliases
shopt -s expand_aliases
source "/common_aliases.sh"

if [[ -f ${CUSTOM_WS}/install/setup.bash ]]
then
    echo "Sourcing custom workspace"
    source "${CUSTOM_WS}/install/setup.bash"
    /copy_dds_configs.sh
else
    echo -e "Custom workspace is not built. Please build it with: \ncolcon build\n"
fi

exec "$@"
