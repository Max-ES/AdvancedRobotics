#! /bin/env python
import math
import time

import rclpy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
import scipy.interpolate
import numpy as np
from scipy.spatial.distance import cdist
from sensor_msgs.msg import LaserScan


def euler_from_quaternion(orientation):
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


class CarController(Node):

    def __init__(self):
        super().__init__("car_controller")
        self.calls_per_second = 100
        self.last_steer_error = 0.0
        self.curr_angle = 0.0
        self.target_angle = 0.0
        self.curr_position = [0.0, 0.0]
        self.lane_index = 1
        self.lookahead_distance = 1.2

        self.publisher_speed_ = self.create_publisher(SpeedCommand, '/actuators/speed', 10)
        self.publisher_steer_ = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized',
                                                      10)
        self.subscriber_filtered_map = self.create_subscription(Odometry, '/sensors/localization/filtered_map',
                                                                self.new_odometry_callback, 10)
        self.subscriber_lidar = self.create_subscription(LaserScan, '/sensors/rplidar/scan',
                                                         self.scanner_callback, 10)
        self.create_timer(1 / self.calls_per_second, self.steer)
        self.create_timer(1 / 20, self.update_target_angle)
        self.map = Map()

    def new_odometry_callback(self, odo: Odometry):
        _, _, self.curr_angle = euler_from_quaternion(odo.pose.pose.orientation)
        self.curr_position = [odo.pose.pose.position.x, odo.pose.pose.position.y]

    def scanner_callback(self, scan: LaserScan):
        scan_results = scan.ranges
        obstacle_detected = self.is_obstacle_on_lane(scan_results, self.map.lanes[self.lane_index])
        other_lane = self.map.lanes[0 if self.lane_index == 1 else 1]
        obstacle_detected_on_other_lane = self.is_obstacle_on_lane(scan_results, other_lane)

        if obstacle_detected and obstacle_detected_on_other_lane:
            print("obstacle on both lanes")
        elif obstacle_detected or obstacle_detected_on_other_lane:
            print(f"obstacle on lane {self.lane_index}")
        else:
            print("no obstacle")

        if obstacle_detected:
            self.lookahead_distance = 0.6
            if obstacle_detected_on_other_lane:
                self.set_speed(0.0)
            else:
                self.lane_index = 0 if self.lane_index == 1 else 1
                print(f"change lane to {self.lane_index}")
        else:
            self.set_speed(0.15)

        if not obstacle_detected and not obstacle_detected_on_other_lane:
            self.lookahead_distance = 1.2

    def is_obstacle_on_lane(self, scan_results: list[float], lane: any, distance_threshold=0.8,
                            obstacle_threshold=0.1, range_deg=50):
        for i, value in enumerate(scan_results):
            degree_deg = i / 2.
            if not (degree_deg < range_deg or degree_deg > 360 - range_deg) or value > distance_threshold:
                continue
            origin = self.curr_position
            distance = value
            x = origin[0] + distance * np.cos(np.deg2rad(degree_deg))
            y = origin[1] + distance * np.sin(np.deg2rad(degree_deg))
            # get the closest point on the lane
            closest_point, _ = lane.closest_point([x, y])
            # get the distance between the closest point and the point on the scan
            distance = math.sqrt((closest_point[0][0] - x) ** 2 + (closest_point[0][1] - y) ** 2)
            if distance < obstacle_threshold:
                print(f"obstacle: {degree_deg}")
                return True
        return False

    def update_target_angle(self):
            lookahead_point, _ = self.map.lanes[self.lane_index].lookahead_point(self.curr_position, self.lookahead_distance)
            # get the angle between the lookahead point (x, y) and the current position (x, y)
            self.target_angle = np.arctan2(lookahead_point[0][1] - self.curr_position[1], lookahead_point[0][0] - self.curr_position[0])

    def steer(self):
        curr_angle = self.curr_angle

        K_p = 1
        K_d = 0.2

        error = self.target_angle - curr_angle
        p = K_p * error
        d = 0 if not self.last_steer_error else K_d * (error - self.last_steer_error) * self.calls_per_second
        angle = p + d
        angle = max(-1.0, min(1.0, angle))  # cap to -1 to 1

        self.last_steer_error = error
        self.set_steer_angle(angle)

    def set_steer_angle(self, angle: float):
        msg_steer = NormalizedSteeringCommand()
        msg_steer.value = angle  # -1 = right; 1 = left; 0 = straight
        self.publisher_steer_.publish(msg=msg_steer)  # publish the target steering angle

    def set_speed(self, speed: float):
        msg = SpeedCommand()
        msg.value = speed
        self.publisher_speed_.publish(msg=msg)


class Lane:

    def __init__(self, support_points):
        self.support_points = support_points
        self.spline_x = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [1]],
                                                      bc_type='periodic')
        self.spline_y = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [2]],
                                                      bc_type='periodic')

    def length(self):
        return self.support_points[:, 0][-1]

    def interpolate(self, param):
        return np.column_stack((self.spline_x(param), self.spline_y(param)))

    def closest_point(self, point, precision=0.001, min_param=0.0, max_param=-1.0):
        step_size = 0.2
        if max_param < 0:
            max_param = self.length()
        closest_param = -1.0

        while step_size > precision:
            params = np.arange(min_param, max_param, step_size)
            points = self.interpolate(params)

            closest_index = cdist([point], points, 'sqeuclidean').argmin()
            closest_param = params[closest_index]
            min_param = max(min_param, closest_param - step_size)
            max_param = min(max_param, closest_param + step_size)
            step_size *= 0.5

        return self.interpolate(closest_param), closest_param

    def lookahead_point(self, point, lookahead_distance):
        closest_point, closest_param = self.closest_point(point)
        return self.interpolate(closest_param + lookahead_distance), closest_param + lookahead_distance


class Map:

    def __init__(self):
        self.lane_1 = np.load("lane1.npy")
        self.lane_2 = np.load("lane2.npy")
        self.lanes = [
            Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
            Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]


def main(args=None):
    rclpy.init(args=args)
    print("initialized")

    controller = CarController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

