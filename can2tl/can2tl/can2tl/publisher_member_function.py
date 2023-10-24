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
import lanelet2

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ros2_socketcan_msgs.msg import FdFrame
from autoware_perception_msgs.msg import TrafficSignalArray
from autoware_perception_msgs.msg import TrafficSignalElement
from autoware_perception_msgs.msg import TrafficSignal
from autoware_auto_planning_msgs.msg import PathWithLaneId

class MinimalPublisher(Node):
    lane_ids = None
    osmMap = None

    def __init__(self):
        super().__init__('minimal_publisher')
        self.osmMap = lanelet2.io.load("/home/chinyikming/ARTC/ARTC-map-project-2023/ARTC-data/lanelet2/ARTC.osm", lanelet2.io.Origin(0,0))
        #autoware launch file already setup topic: /perception/traffic_light_recognition/traffic_signals, so we can just reuse it
        self.publisher_ = self.create_publisher(
            TrafficSignalArray, 
            'perception/traffic_light_recognition/traffic_signals', 
            1) 
        timer_period = 0.1  # 0.1 seconds = 10Hz
        self.canSub = self.create_subscription(
            FdFrame,
            'canData',
            self.canSub_listener_callback,
            0)
        self.laneSub = self.create_subscription(
            PathWithLaneId,
            'planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.laneSub_listener_callback,
            0) # assume the lane has decent distance for buffering
        #self.timer = self.create_timer(timer_period, self.listener_callback)
        self.canSub  # prevent unused variable warning
        self.laneSub  # prevent unused variable warning

    def laneSub_listener_callback(self, msg):
        self.lane_ids = msg.points[0].lane_ids
        #print(self.lane_ids)

    def canSub_listener_callback(self, msg):
        state = self.getState(msg.data)
        intersectionId = self.getIntersectionId(msg.data)
        trafficLightState = self.getTrafficLightState(msg.data)
        trafficLightRemainSeconds = self.getTrafficLightRemainSeconds(msg.data)
        counter = self.getCounter(msg.data)

        trafficLightId = None
        if self.osmMap != None and self.lane_ids != None:
            for lane_id in self.lane_ids:
                trafficLightId = self.getTrafficLightId(lane_id)
                if trafficLightId != None: # assume each lane only has one traffic light
                    break

        # if trafficLightId != None:
            # trafficSignals = self.trafficSignalsGen(trafficLightId)
        trafficSignals = self.trafficSignalsGen(400004)
        self.publisher_.publish(trafficSignals)
        print("pub")
        
        #self.get_logger().info(f'state: {state}')
        #self.get_logger().info(f'intersectionId: {intersectionId}')
        #self.get_logger().info(f'trafficLightState: {trafficLightState}')
        #self.get_logger().info(f'trafficLightRemainSeconds: {trafficLightRemainSeconds}')
        #self.get_logger().info(f'counter: {counter}')

    def getTrafficLightId(self, laneId):
        tlId = None
        for elem in self.osmMap.laneletLayer:
            #print("elem_id:",elem.id, "laneId:", laneId)
            if elem.regulatoryElements != [] and elem.id == laneId:
                tl = elem.regulatoryElements.pop()
                tlId = tl.parameters["refers"].pop().id
                print("match!!!")
                break
        return tlId

    def stampGen(self, frame_id = ''):
        stamp = Time()
        t = self.get_clock().now()
        stamp = t.to_msg()
        #stamp.sec = int(t - math.floor(t))
        #hdr.stamp.nanosec = int(math.floor((t - math.floor(t)) * 10000000))
        #hdr.stamp.sec = 1
        # stamp.sec = int(t)
        # stamp.nanosec = 0
        return stamp

    def trafficSigEleGen(self, state):
        tse = TrafficSignalElement()
        tse.color = state[0]
        tse.shape = state[1]
        tse.status = state[2]
        tse.confidence = state[3]
        return tse

    def trafficSignalGen(self, tl_id, state):
        ts = TrafficSignal()
        ts.traffic_signal_id = tl_id
        ts.elements.append(self.trafficSigEleGen(state))
        return ts

    def trafficSignalsGen(self, tl_id, state = (1, 1, 2, 1.0)):
        stamp = self.stampGen()
        ts = self.trafficSignalGen(tl_id, state)
        trafficSignals = TrafficSignalArray()
        trafficSignals.stamp = stamp
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
