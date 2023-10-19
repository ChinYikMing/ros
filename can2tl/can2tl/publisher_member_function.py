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

import math
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ros2_socketcan_msgs.msg import FdFrame
from autoware_auto_perception_msgs.msg import TrafficSignalArray
from autoware_auto_perception_msgs.msg import TrafficLight
from autoware_auto_perception_msgs.msg import TrafficSignal

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        #autoware launch file already setup topic: /perception/traffic_light_recognition/traffic_signals, so we can just reuse it
        self.publisher_ = self.create_publisher(TrafficSignalArray, 'perception/traffic_light_recognition/traffic_signals', 0) 
        timer_period = 0.1  # 0.1 seconds = 10Hz
        self.subscription = self.create_subscription(
            FdFrame,
            'canData',
            self.listener_callback,
            0)
        #self.timer = self.create_timer(timer_period, self.listener_callback)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        state = self.getState(msg.data)
        intersectionId = self.getIntersectionId(msg.data)
        trafficLightState = self.getTrafficLightState(msg.data)
        trafficLightRemainSeconds = self.getTrafficLightRemainSeconds(msg.data)
        counter = self.getCounter(msg.data)

        trafficSignals = self.trafficSignalsGen(120008)
        self.publisher_.publish(trafficSignals)
        
        self.get_logger().info(f'state: {state}')
        self.get_logger().info(f'intersectionId: {intersectionId}')
        self.get_logger().info(f'trafficLightState: {trafficLightState}')
        self.get_logger().info(f'trafficLightRemainSeconds: {trafficLightRemainSeconds}')
        self.get_logger().info(f'counter: {counter}')

        #self.publisher_.publish(msg)

    def headerGen(self, frame_id = ''):
        hdr = Header()
        hdr.stamp = Time()
        #t = time.time()
        #hdr.stamp.sec = int(t - math.floor(t))
        #hdr.stamp.nanosec = int(math.floor((t - math.floor(t)) * 10000000))
        hdr.stamp.sec = 1
        hdr.stamp.nanosec = 2
        hdr.frame_id = frame_id
        return hdr

    def trafficLightGen(self, state):
        tl = TrafficLight()
        tl.color = state[0]
        tl.shape = state[1]
        tl.status = state[2]
        tl.confidence = state[3]
        return tl

    def trafficSignalGen(self, tl_id, state):
        ts = TrafficSignal()
        ts.map_primitive_id = tl_id
        ts.lights.append(self.trafficLightGen(state))
        return ts

    def trafficSignalsGen(self, tl_id, state = (1, 5, 16, 1.0)):
        hdr = self.headerGen()
        ts = self.trafficSignalGen(tl_id, state)
        trafficSignals = TrafficSignalArray()
        trafficSignals.header = hdr
        trafficSignals.signals.append(ts)
        return trafficSignals

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
