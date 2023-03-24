# Docker container image build for running MoveIt Studio with
# a custom overlay image
#
# Example build command (with defaults):
#
# docker build -f .docker/Dockerfile .
#

# Specify the MoveIt Studio release to build on top of. See,
# https://hub.docker.com/r/picknikciuser/moveit-studio-binaries/tags
ARG MOVEIT_STUDIO_BASE_IMAGE

#####################################################
# Starting from the specified MoveIt Studio release #
#####################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_STUDIO_BASE_IMAGE} as base

# The location of the user's custom workspace inside the container
ARG CUSTOM_WS=/opt/custom_ws
ENV CUSTOM_WS $CUSTOM_WS

# Copy site configuration packages and and dependencies
RUN mkdir -p ${CUSTOM_WS}/src
COPY ./src $CUSTOM_WS/src
WORKDIR $CUSTOM_WS

# Install additional ROS dependencies
# NOTE: The "OVERLAY_WS" contains MoveIt Studio binary packages and the source file.
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . "${OVERLAY_WS}/install/setup.sh" && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src

#################################################
# Target for compiled, deployable release image #
#################################################
FROM base as release

WORKDIR $CUSTOM_WS

# Compile the workspace
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/root/.ccache \
    . "${OVERLAY_WS}/install/setup.sh" && \
    colcon build

# Add the custom entrypoint
COPY .docker/entrypoint-release.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/usr/bin/bash"]

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
# Expected to mount workspace and compile/test it
# inside of the resulting container.
FROM base as dev

# The location of the user's workspace inside the container
ARG CUSTOM_WS=/opt/custom_ws
ENV CUSTOM_WS $CUSTOM_WS
WORKDIR $CUSTOM_WS

# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    less \
    gdb \
    gdbserver \
    ros-humble-rclcpp-dbgsym

# Add the developer entrypoint
COPY .docker/entrypoint-dev.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc && \
    echo "set +e" >> ~/.bashrc
CMD ["/usr/bin/bash"]
