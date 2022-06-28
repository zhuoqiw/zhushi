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

# Copy from open
COPY --from=opencv /setup /

# Copy from pylon
COPY --from=pylon /setup /

# Setup environment
ENV PYLON_ROOT=/opt/pylon

# Setup ldconfig
RUN ldconfig
