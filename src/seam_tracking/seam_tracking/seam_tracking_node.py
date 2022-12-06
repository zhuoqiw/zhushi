"""
ROS node to locate seams from points, supports source code plugin.

A python ROS node to subscribe from upstream topic.
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

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, Log

class SeamTracking(Node):
    """
    ROS node to locate seam and maintain plugins.

    A python ROS node to subscribe from upstream topic.
    Apply a serial of customizable plugins to locate seam.
    """

    def __init__(self):
        Node.__init__(self, 'seam_tracking_node')

        self.cli1 = self.create_client(Trigger, '/camera_pylon_node_l/start')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service(camL) not available, waiting again...')

        self.cli2 = self.create_client(Trigger, '/camera_pylon_node_r/start')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service(camR) not available, waiting again...')

        self.cli3 = self.create_client(Trigger, '/motor_encoder_node/zero')
        while not self.cli3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service(motor zero) not available, waiting again...')
        
        self.cli4 = self.create_client(Trigger, '/motor_encoder_node/scan')
        while not self.cli4.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service(motor scan) not available, waiting again...')

        self.cli5 = self.create_client(SetParameters, '/gpio_raspberry_node/set_parameters')
        while not self.cli5.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gpio not available, waiting again...')

        self.cli6 = self.create_client(Trigger, '/camera_pylon_node_l/stop')
        while not self.cli6.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service(camL) not available, waiting again...')

        self.cli7 = self.create_client(Trigger, '/camera_pylon_node_r/stop')
        while not self.cli7.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service(camR) not available, waiting again...')

        self.srv = self.create_service(Trigger, '~/start', self.add_two_ints_callback)

        self.get_logger().info('Initialized successfully')
    
    def add_two_ints_callback(self, request, response):
        t = Trigger.Request()
        self.cli1.call_async(t)
        self.cli2.call_async(t)
        self.cli3.call_async(t)
        time.sleep(5)
        r = SetParameters.Request()
        v = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        r.parameters = [Parameter(name='laser', value=v), Parameter(name='trigger', value=v)]
        self.cli5.call_async(r)
        self.cli4.call_async(t)
        time.sleep(5)
        v = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
        r.parameters = [Parameter(name='laser', value=v), Parameter(name='trigger', value=v)]

        self.cli6.call_async(t)
        self.cli7.call_async(t)
        # rclpy.spin_until_future_complete(self, f)
        # f = self.cli2.call_async(t)
        # rclpy.spin_until_future_complete(self, f)
        # self.cli3.call(t)
        # self.cli4.call(t)
        response.success = True
        return response

    def __del__(self):
        self.get_logger().info('Destroyed successfully')

def main(args=None):
    rclpy.init(args=args)

    seam_tracking = SeamTracking()

    try:
        rclpy.spin(seam_tracking)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        # PIPE_OUT.close()
        # child.join()
        seam_tracking.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
