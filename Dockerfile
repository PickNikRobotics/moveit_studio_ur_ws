# Docker image for extending MoveIt Studio with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Studio release to build on top of.
ARG MOVEIT_STUDIO_BASE_IMAGE
ARG USERNAME=studio-user
ARG USER_UID=1000
ARG USER_GID=1000

#####################################################
# Starting from the specified MoveIt Studio release #
#####################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_STUDIO_BASE_IMAGE} as base

# hadolint ignore=DL3002
USER root

# Create a non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_DIR=/home/${USERNAME}/user_overlay_ws
ENV USER_DIR=${USER_DIR}
RUN mkdir -p ${USER_DIR}/src ${USER_DIR}/build ${USER_DIR}/install ${USER_DIR}/log
COPY ./src ${USER_DIR}/src

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_DIR
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME} && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    mkdir -p \
      /home/${USERNAME}/.ccache \
      /home/${USERNAME}/.config \
      /home/${USERNAME}/.ignition \
      /home/${USERNAME}/.ros/log && \
    chown -R $USER_UID:$USER_GID /home/${USERNAME} /opt/overlay_ws/

# Install additional dependencies
# You can also add any necessary apt-get install, pip install, etc. commands at this point.
# NOTE: The /opt/overlay_ws folder contains MoveIt Studio binary packages and the source file.
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/overlay_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src

#########################################
# Target for compiled, deployable image #
#########################################
FROM base as user-overlay

ARG USERNAME
USER ${USERNAME}
ARG USER_DIR=/home/${USERNAME}/user_overlay_ws
ENV USER_DIR=${USER_DIR}

# Compile the workspace
WORKDIR $USER_DIR
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/home/${USERNAME}/.ccache \
    . /opt/overlay_ws/install/setup.sh && \
    colcon build

# Add the custom entrypoint
COPY ./entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh && set +e" >> ~/.bashrc
CMD ["/usr/bin/bash"]

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
FROM base as user-overlay-dev

USER root
# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        gdb \
        nano

USER ${USERNAME}
ARG USERNAME
ARG USER_DIR=/home/${USERNAME}/user_overlay_ws
ENV USER_DIR=${USER_DIR}

# Add the dev entrypoint
COPY ./entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh && set +e" >> ~/.bashrc
CMD ["/usr/bin/bash"]
