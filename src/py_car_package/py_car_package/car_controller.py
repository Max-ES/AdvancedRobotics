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
import os
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import time
import numpy as np
from scipy.interpolate import CubicSpline
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from std_msgs.msg import Float64

calls_per_second = 100

def create_spline_interpolation(input_array):
    # Extract arc_length, x, and y
    #input_array()
    arc_length = input_array[:, 0]
    x = input_array[:, 1]
    y = input_array[:, 2]
    arc_length = [e for i, e in enumerate(arc_length) if i % 100 == 0 ]
    x = [e for i, e in enumerate(x) if i % 100 == 0 ]
    y = [e for i, e in enumerate(y) if i % 100 == 0 ]

    # Create cubic splines
    spline_x = CubicSpline(arc_length, x)
    spline_y = CubicSpline(arc_length, y)

    # Sample the spline at 1 cm intervals
    arc_length_sampled = np.arange(arc_length[0], arc_length[-1], 0.01)
    x_sampled = spline_x(arc_length_sampled)
    y_sampled = spline_y(arc_length_sampled)

    return arc_length_sampled, x_sampled, y_sampled

def create_line_strip_marker(points):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = 0.01  # Width of the line
    marker.color.a = 1.0  # Alpha
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    for point in points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = 0.0
        marker.points.append(p)

    return marker




class CarController(Node):
    store = {} # stores last error values, last_called timestamps etc.

    def __init__(self):
        super().__init__('car_controller')
        self.get_logger().info('Started car package')
        self.target_spped = None
        self.set_target_speed(0.0)
        self.i = 0
        self.current_wheel_based_speed = 0
        self.speed_controller_output = 0
        self.speed_history: list[tuple[float, float, float, float]] = [] # time, current speed, target speed, controller output
        #timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.drive_in_sin_curve)
        self.publisher_steer_ = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized', 10)
        self.publisher_speed_ = self.create_publisher(SpeedCommand, '/actuators/speed', 10)
        #self.publisher_target_speed_ = self.create_publisher(SpeedCommand, '/autominy_msgs/SpeedCommand', 10)
        self.publisher_steering_angle_ = self.create_publisher(Float64, '/steering_meassured', 10)
        self.publisher_target_angle_in_room = self.create_publisher(NormalizedSteeringCommand, 'autominy_msgs/SteeringCommand', 10)
        
        self.subscriber_filtered_map = self.create_subscription(Odometry, '/sensors/localization/filtered_map', self.filtered_map_callback, 10)
        self.create_timer(1 / calls_per_second, self.process)
        #self.subscriber_target_speed_ = self.create_subscription(SpeedCommand, '/autominy_msgs/SpeedCommand', self.pid_speed_controller, 10)
        #self.steering_subscriber = self.create_subscription(NormalizedSteeringCommand, 'autominy_msgs/SteeringCommand', self.steer ,10)
        self.last_callback_time = None
        self.last_steer_time = 0
        self.last_error = 0
        self.last_odo = None

        self.lane1 = np.load(os.path.join(os.path.dirname(__file__),"lane1.npy"))
        print(list(self.lane1))

        self.lane2 = np.load(os.path.join(os.path.dirname(__file__),"lane2.npy"))

        self.last_speed_change = 0
        self.current_speed_sequence_step = 0
        self.speed_error_sum = 0
        self.speed_last_error = 0

        


    def process(self):
        #self.set_target_anlge_in_room(0.0)
        input_array = self.lane1  # Replace with your actual data
        arc_length_sampled, x_sampled, y_sampled = create_spline_interpolation(input_array)
        points = np.column_stack((x_sampled, y_sampled))
        marker = create_line_strip_marker(points)
        self.publish_marker(marker)

        self.speed_change_sequence()
        self.set_steer_angle(0.8)
        self.pid_speed_controller()

    def publish_marker(self, marker: Marker):
        publisher_map_ = self.create_publisher(Marker, '/visualization_msgs/Marker', 10)
        publisher_map_.publish(marker)
        
        
    def filtered_map_callback(self, odo: Odometry):
        self.current_wheel_based_speed = odo.twist.twist.linear.x
        self.meassure_and_publish_steering_angle(odo)

    def meassure_and_publish_steering_angle(self, odo: Odometry):
        current_time = time.time()
        _, _, self.curr_angle = self.euler_from_quaternion(odo.pose.pose.orientation)
        
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
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed
    
    def set_speed(self, speed: float):
        msgSpeed = SpeedCommand()
        msgSpeed.value = speed
        self.publisher_speed_.publish(msg=msgSpeed)

    def set_steer_angle(self, angle: float):
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = angle # -1 = right; 1 = left; 0 = straight
        self.publisher_steer_.publish(msg=msgSteer) # publish the target steering angle

    def set_target_anlge_in_room(self, angle: float):
        msgSteer = NormalizedSteeringCommand()
        msgSteer.value = angle
        self.publisher_target_angle_in_room.publish(msgSteer)
    
    def forward(self):
        self.set_speed(0.3)
        self.set_steer_angle(0.0)

    def pid_speed_controller(self):
        current_speed = self.current_wheel_based_speed
        # PID coefficients
        # calculated coefficients:
        #K_p = 7.69
        #K_i = 76.9 * (1 / calls_per_second)
        #K_d = 0.19 * calls_per_second
        # coefficients that work:
        K_p = 0.4
        K_i = 3.5 * (1 / calls_per_second)
        K_d = 0.001 * calls_per_second 

        # Calculate error
        error = self.target_speed - current_speed

        # Proportional term
        P_out = K_p * error

        # Integral term
        self.speed_error_sum += P_out #error
        I_out = K_i * self.speed_error_sum

        # Derivative term
        delta_error = error - self.speed_last_error
        D_out = K_d * delta_error

        # Calculate total output
        output = P_out + I_out + D_out
        if (output < 0):
            output = 0.0 # don't drive backwards
        print(output)
        print(f"P: {P_out}, I: {I_out}, D: {D_out}")
        print(f"v: {current_speed}, t: {self.target_speed}, error: {error}, P_out: {P_out} out: {output}")
        self.set_speed(output)

        self.speed_controller_output = output
        # Update last error
        self.speed_last_error = error


    def speed_change_sequence(self, sequence=[0.2, 0.5, 0.8, 0.2], step_length_in_seconds=3):
        current_time = time.time()
        if current_time - self.last_speed_change >= step_length_in_seconds:
            #changeSpeed
            self.current_speed_sequence_step += 1
            if self.current_speed_sequence_step >= len(sequence):
                self.current_speed_sequence_step = 0

            new_speed = sequence[self.current_speed_sequence_step]
            if self.current_speed_sequence_step == 0 and len(self.speed_history) > 0:
                # save the speed plot, when step 0 is reached again
                plot_speed(self.speed_history)
            print(f"set target speed to {new_speed}")
            
            self.target_speed = new_speed

            self.last_speed_change = current_time

        self.speed_history.append((time.time(), self.current_wheel_based_speed, sequence[self.current_speed_sequence_step], self.speed_controller_output))
        #print(self.speed_history[-10:])
        

    def drive_in_sin_curve(self):
        i = self.store.setdefault("i", 0)
        self.set_steer_angle(math.sin(self.i))
        self.set_speed(.3)
        self.store["i"] += .01


    def steer(self, steering_command):
        current_time = time.time() * 1000
        self.last_steer_time = 0 if not self.last_callback_time else self.last_steer_time
        calls_per_second = 100
        if current_time - self.last_steer_time >= 1000 / calls_per_second:
            target_angle = steering_command.value
            curr_angle = self.curr_angle

            K_p = 1
            K_d = 0.2

            error = target_angle - curr_angle
            p = K_p * error
            d = 0 if not self.last_error else K_d * (error - self.last_error) * calls_per_second
            angle = p + d
            angle = max(-1.0, min(1.0, angle)) # cap to -1 to 1

            self.last_error = error
            self.last_steer_time = current_time
            self.set_steer_angle(angle)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CarController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


def plot_speed(hist):
    time, current_speed, target_speed, controller_output = zip(*hist)

    # Plotting the data
    plt.figure(figsize=(10, 6))
    plt.plot(time, current_speed, label='Current Speed (m/s)')
    plt.plot(time, target_speed, label='Target Speed (m/s)', linestyle='--')
    plt.plot(time, controller_output, label='Controller Output (m/s)', linestyle='--')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig("plot.png")


if __name__ == '__main__':
    main()
