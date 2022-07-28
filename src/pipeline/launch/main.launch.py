"""
Generate a launch description.

This launch configuration gathers a bunch of node into a pipeline:
It parses the underlay config: .params.yaml and overlay config: params.yaml.
The overlay is updated to the underlay.
Several computation intensive nodes employee more workers.
Image related topics are passed around via intra-process communication.
Only pointers to image are copied to minimize CPU consumption.
"""

# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with a component."""
    configFile = os.path.join(
        get_package_share_directory('camera_pylon'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        params = yaml.safe_load(file)['camera_pylon_node']['ros__parameters']

    params['workers'] = 2
    params['serial'] = '40146410'
    camera_pylon_node_l = Node(
        package='camera_pylon',
        executable='camera_pylon_node',
        name='camera_pylon_node_l',
        parameters=[params])

    params['workers'] = 2
    params['serial'] = '40146429'
    camera_pylon_node_r = Node(
        package='camera_pylon',
        executable='camera_pylon_node',
        name='camera_pylon_node_r',
        parameters=[params])

    # container = ComposableNodeContainer(
    #     name='pipeline_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         camera_pylon_node_l,
    #         resize_image_node,
    #         rotate_image_node,
    #         laser_line_center_node,
    #         laser_line_filter_node,
    #         line_center_reconstruction_node])

    # seam_tracking_node = Node(
    #     package='seam_tracking',
    #     executable='seam_tracking_node',
    #     remappings=[('~/pnts', '/line_center_reconstruction_node/pnts')],
    #     parameters=[params['seam_tracking_node']])

    # modbus_node = Node(
    #     package='modbus',
    #     executable='modbus_node',
    #     parameters=[params['modbus_node']])

    # gpio_raspberry_node = Node(
    #     package='gpio_raspberry',
    #     executable='gpio_raspberry_node',
    #     parameters=[params['gpio_raspberry_node']])

    # config_tis_node = Node(
    #     package='config_tis',
    #     executable='config_tis_node',
    #     on_exit=launch.actions.Shutdown())

    return launch.LaunchDescription([
        camera_pylon_node_l,
        camera_pylon_node_r])
