# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from ros2_socketcan_msgs.msg import FdFrame

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(FdFrame, 'topic', 10)
        timer_period = 0.5  # seconds
        self.subscription = self.create_subscription(
            FdFrame,
            'canData',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        state = self.getState(msg.data)
        intersectionId = self.getIntersectionId(msg.data)
        trafficLightState = self.getTrafficLightState(msg.data)
        trafficLightRemainSeconds = self.getTrafficLightRemainSeconds(msg.data)
        counter = self.getCounter(msg.data)
        
        self.get_logger().info(f'state: {state}')
        self.get_logger().info(f'intersectionId: {intersectionId}')
        self.get_logger().info(f'trafficLightState: {trafficLightState}')
        self.get_logger().info(f'trafficLightRemainSeconds: {trafficLightRemainSeconds}')
        self.get_logger().info(f'counter: {counter}')

        #self.publisher_.publish(msg)

    def getState(self, data):
        return data[0]

    def getIntersectionId(self, data):
        return data[3]

    def getTrafficLightState(self, data):
        return data[4]

    def getTrafficLightRemainSeconds(self, data):
        return (data[6] * 256 + data[5]) * 0.1

    def getCounter(self, data):
        return data[7]

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
