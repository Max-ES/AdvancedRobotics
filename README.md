# AdvancedRobotics

## Setup
Make sure you followed the [install instructions](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html).
In .bashrc should be the following lines:
```
export ROS_DOMAIN_ID=31  # number of the car - 100
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /home/max/autominy/catkin_ws/install/setup.bash  # replace by your autominy install path
```

Root of the workspace == the cloned /AdvancedRobotics directroy

### build workspace
In root (/AdvancedRobotics)
```
colcon build
```
If you get the error message: 
>SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.

...you probably have to downgrade your setuptools version:
```
pip install setuptools==58.2.0
```

### run workspace
in root:
```
ros2 run py_car_package car_controller
```

#### Add workspace paths to bashrc to source automatically like:
```
source /home/max/AdvancedRobotics/install/setup.bash
```


## How Tos
### copy folder from car to pc (over ssh)
```
scp -r ros@192.168.43.131:~/<filename> ./
```

### record ros topic
```
ros2 bag record <topic_name>
```

### create a package
```
ros2 pkg create --build-type ament_python <package_name>
```


