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


        self.publisher_steer_ = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized', 10)
        self.publisher_speed_ = self.create_publisher(SpeedCommand, '/actuators/speed', 10)
        self.publisher_steering_angle_ = self.create_publisher(Float64, '/steering_meassured', 10)
        
        self.fake_publisher = self.create_publisher(NormalizedSteeringCommand, 'autominy_msgs/SteeringCommand', 10)

        self.subscriber_ = self.create_subscription(Odometry, '/sensors/localization/filtered_map', self.meassure_and_publish_steering_angle, 10)
        self.steering_subscriber = self.create_subscription(NormalizedSteeringCommand, 'autominy_msgs/SteeringCommand', self.steer ,10)
        self.last_callback_time = None
        self.last_steer_time = 0
        self.last_error = 0
        self.last_odo = None

        self.forward()
        

    def meassure_and_publish_steering_angle(self, odo: Odometry):
        current_time = time.time()
        
        _, _, self.curr_angle = self.euler_from_quaternion(odo.pose.pose.orientation)
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = 0.0 # publish the target angle in the room
        self.fake_publisher.publish(msg=msgSteer)
        
        if self.last_odo is None:
            self.last_odo = odo
            self.last_callback_time = current_time
            return
        
        # Check if one second has passed since the last callback
        if current_time - self.last_callback_time >= 1:
            steering_angle = self.calculate_steering_angle(self.last_odo, odo, 0.27)
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
        # Calculate the arc length s, which is the distance traveled
        euclidian_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        r = euclidian_dist / (np.sin(delta_yaw/2)*2)
        travelled_distance = r * delta_yaw # Bogenlänge - just for fun

        # Calculate the steering angle delta
        # see: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
        # "From the triangle in Fig. 22 we find ..."
        steering_angle = math.atan(wheelbase/r) 
        
        return steering_angle
    
    def forward(self):
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = 0.0 # -1 = right; 1 = left; 0 = straight

        self.publisher_steer_.publish(msg=msgSteer) # publish the target steering angle

        msgSpeed = SpeedCommand()
        msgSpeed.value = 0.3
        self.publisher_speed_.publish(msg=msgSpeed)
        

    def drive_in_sin_curve(self):
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = math.sin(self.i)
        self.publisher_steer_.publish(msg=msgSteer)

        msgSpeed = SpeedCommand()
        msgSpeed.value = .3
        self.publisher_speed_.publish(msg=msgSpeed)

        self.i += .01

    def steer(self, steering_command):
        current_time = time.time() * 1000
        self.last_steer_time = 0 if not self.last_callback_time else self.last_steer_time
        calls_per_second = 100
        if current_time - self.last_steer_time >= 1000 / calls_per_second:
            msgSteer = NormalizedSteeringCommand()

            target_angle = steering_command.value
            curr_angle = self.curr_angle

            K_p = 1
            K_d = 0.2

            # gut : 1 0.2 100/s
            # bad: 10, 2 100/s (left right left right)
            # slow: 0.2 0 100/s (too large radius)

            error = target_angle - curr_angle
            p = K_p * error
            d = 0 if not self.last_error else K_d * (error - self.last_error) * calls_per_second
            angle = p + d
            angle = max(-1.0, min(1.0, angle)) # cap to -1 to 1
            print(target_angle, curr_angle, p, angle)
            msgSteer.value = angle

            self.last_error = error
            self.last_steer_time = current_time
            self.publisher_steer_.publish(msg=msgSteer) # set the steering angle of the car



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
