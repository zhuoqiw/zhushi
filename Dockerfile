# Global args
# Add a non-root user to a container
# For more information, visit: https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Ubuntu LTS release: 20.04, 22.04
ARG UBUNTU_VERSION

# ROS code name: galactic, humble
ARG ROS_DISTRO

# Use opencv
FROM zhuoqiw/ros-opencv:4.5.5-${UBUNTU_VERSION} AS opencv

# Use pylon
FROM zhuoqiw/ros-pylon:6.2.0-${UBUNTU_VERSION} AS pylon

# Base image
FROM ros:${ROS_DISTRO} AS base

# Enable root user access ros
# RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home $USERNAME

# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Copy from opencv
COPY --from=opencv /setup /

# Copy from pylon
COPY --from=pylon /setup /

# Setup environment
ENV PYLON_ROOT=/opt/pylon

# Install dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    gdb \
    nano \
    gpiod \
    libgpiod-dev \
    && rm -rf /var/lib/apt/lists/*

# Setup ldconfig
RUN ldconfig

USER ros
