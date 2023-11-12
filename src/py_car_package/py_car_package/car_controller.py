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
from nav_msgs.msg import Odometry
import numpy as np
import time

from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from std_msgs.msg import Float64

class CarController(Node):

    def __init__(self):
        super().__init__('car_controller')
        self.get_logger().info('Started car package')

        self.i = 0
        #timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.drive_in_sin_curve)

        #self.forward()

        self.publisher_steer_ = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized', 10)
        self.publisher_speed_ = self.create_publisher(SpeedCommand, '/actuators/speed', 10)
        self.publisher_steering_angle_ = self.create_publisher(Float64, '/steering_meassured', 10)
        
        self.subscriber_ = self.create_subscription(Odometry, '/sensors/localization/filtered_map', self.meassure_and_publish_steering_angle, 10)
        self.last_callback_time = None
        self.last_odo = None
        

    def meassure_and_publish_steering_angle(self, odo: Odometry):
        current_time = time.time()
        
        if self.last_odo is None:
            self.last_odo = odo
            self.last_callback_time = current_time
            return
        
        # Check if one second has passed since the last callback
        if current_time - self.last_callback_time >= 1:
            steering_angle = self.calculate_steering_angle(self.last_odo, odo, 0.27)
            print(steering_angle, math.degrees(steering_angle))
            msg = Float64()
            msg.data = steering_angle
            self.publisher_steering_angle_.publish(msg)
            self.last_odo = odo
            self.last_callback_time = current_time


    def euler_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def calculate_steering_angle(self, odometry1: Odometry, odometry2: Odometry, wheelbase: float) -> float:
        x1, y1 = odometry1.pose.pose.position.x, odometry1.pose.pose.position.y
        x2, y2 = odometry2.pose.pose.position.x, odometry2.pose.pose.position.y

        _, _, yaw1 = self.euler_from_quaternion(odometry1.pose.pose.orientation)
        _, _, yaw2 = self.euler_from_quaternion(odometry2.pose.pose.orientation)
        
        # Calculate change in orientation
        delta_yaw = yaw2 - yaw1
        # Normalize delta_yaw to [-pi, pi]
        delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Calculate the arc length s, which is the distance traveled
        # s should be the length of the driven bow (Bogen), not the chord (Sehne)
        # possible solution: assuming the euclidian distance between the two points is the diameter of a circle and the radius of the circle is r = d/2
        euclidian_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        s = 1/2 * math.pi * euclidian_dist
        
        # Calculate the radius of curvature R
        R = s / delta_yaw
        # Calculate the steering angle delta
        # see: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
        # "From the triangle in Fig. 22 we find ..."
        steering_angle = math.atan(wheelbase/R) 
        
        return steering_angle
    
    def forward(self):
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = 0 # -1 = right; 1 = left; 0 = straight

        self.publisher_steer_.publish(msg=msgSteer)

        msgSpeed = SpeedCommand()
        msgSpeed.value = .3
        self.publisher_speed_.publish(msg=msgSpeed)
        

    def drive_in_sin_curve(self):
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
