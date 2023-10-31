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
import rclpy
from rclpy.node import Node

from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

class CarController(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Started car package')

        self.i = 0
        timer_period = 0.01  # seconds

        self.publisher_steer_ = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized', 10)
        self.publisher_speed_ = self.create_publisher(SpeedCommand, '/actuators/speed', 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = math.sin(self.i)
        self.publisher_steer_.publish(msg=msgSteer)

        msgSpeed = SpeedCommand()
        msgSpeed.value = .3
        self.publisher_speed_.publish(msg=msgSpeed)

        self.i += .01



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CarController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
