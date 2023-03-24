#!/bin/bash

# Custom entrypoint to start when running a release image.

set -e

source "${CUSTOM_WS}/install/setup.bash"

/copy_dds_configs.sh

exec "$@"
