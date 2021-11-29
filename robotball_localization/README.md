# robotball_localization

A package for robot localization.

At the moment, localization of the robot is provided by [Pozyx](https://www.pozyx.io/creator), and Kalman filter is used for smoothing and velocity estimation.
Other localization methods might be available in the future. Here you can also find a node for odometry calibration.

### Prerequisites
ROS interface for Pozyx must be present for this package to work. Simply clone the repository to ROS workspace and build with catkin:
```shell script 
$ cd ~/catkin_ws/src
$ git clone git@github.com:larics/pozyx_ros.git
$ cd pozyx_ros
$ git checkout creator
$ catkin build
```

### Usage
Specify anchor names and positions in [anchors.yaml](robotball_localization/params/anchors.yaml).

To start localization on one robot (this should be used only for debuging and development):
`roslaunch robotball_localization temp_local.launch`

To start localization on multiple robots:
- Specify robot names in [tags.yaml](robotball_localization/params/tags.yaml).
- Start the scheduler on your computer. It will make sure there is no interference when robots request positioning for themselves. `roslaunch robotball_localization remote_scheduler.laucnh`
- Start the localization on each robot: `roslaunch robotball_localization localize.launch`
