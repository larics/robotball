# robotball

Packages for spherical robots developed by LARICS and TU Delft.

This repository currently consists of following packages:
- `robotball_arduino`: Arduino files required for driving the robot.
- `robotball_control`: Automatic control of robots (PID controllers, velocity trackers, ...)
- `robotball_driver`: Driver acting as a basic interface between Arduino and ROS, including joystick control.
- `robotball_localization`: External localization and state estimation (Kalman filter, Pozyx positioning, ...)
- `robotball_msgs`: Custom ROS messages.
- `robotball_simulation`: Simple simulator for testing.

For detailed information about each package, please look at their respective READMEs.

## Before you start
### Requirements
- Ubuntu 18.04 or Ubuntu 20.04
- ROS Melodic or ROS Noetic
- ROS packages, can be installed with `sudo apt install ros-noetic-<package>`.
  - `joy`
  - `rosserial`
  - `teleop-twist-joy`
  - `teleop-twist-keyboard`
  - `tf-conversions`
  - `twist-mux`
- Python 3.x
- Python libraries, can be installed with `python3 -m pip install <library>`.
  - `numpy`
  - `pypozyx`
  - `pyserial`
  - `scipy`

### Installation
1. Install all the requirements.
1. Create a new catkin workspace or use an existing one. Most commonly this is `~/catkin_ws/`
1. Clone the repository in the `src` folder of your workspace.
    ```shell script
    $ cd <path_to_your_ws>/src/
    $ git clone git@github.com:larics/robotball.git
    ```
1. Build everything using catkin_tools (recommended):
    ```shell script
    $ catkin build
    ```
   or catkin_make (not recommended):
   ```shell script
   $ cd <path_to_your_ws>
   $ catkin_make
   ```
1. Set up `.bashrc` on your robot:
    ```shell script
    # ROS WORKSPACES -> Standard ROS stuff.
    source /opt/ros/noetic/setup.bash
    source /home/raspi/catkin_ws/devel/setup.bash
    
    # Set up ROS for remote master. 
    # Robot's IP is automatically determined.
    # Master's IP needs to be set manually.
    export ROS_IP=$(ip -o route get to 8.8.8.8 | sed -n 's/.*src \([0-9.]\+\).*/\1/p')
    export ROS_MASTER_URI=http://192.168.140.154:11311
    export NAMESPACE="robot_$(echo $HOSTNAME | sed 's/[^0-9]*//g')"
    
    # Useful aliases and custom functions.
    source /home/raspi/catkin_ws/src/robotball/shell_additions/aliases.sh
    source /home/raspi/catkin_ws/src/robotball/shell_additions/git_scripts.sh
    source /home/raspi/catkin_ws/src/robotball/shell_additions/usb_setup.sh
    ```
   Don't forget to re-source .bashrc to apply changes.
6. Set up symlinks for Arduino and Pozyx.
   1. Connect only the Arduino and run `usb_setup_arduino`.
   2. Disconnect Arduino and connect Pozyx. Then run `usb_setup_pozyx`.
   3. Reconnect Pozyx and reboot.
7. Check individual sub-packages for additional instructions.
 
## Usage
In general:
1. If needed, make changes in Arduino sketch.
2. Compile and upload. (See [Arduino README](robotball_arduino/README.md))
3. Start essential things on the central computer
   1. `roscore`
   1. `rosrun joy joy_node`
   1. `rosrun rqt_reconfigure rqt_reconfigure`
   1. `rviz`
4. Start the driver on the robot: `roslaunch robotball_driver driver.launch`
5. Start the Pozyx scheduler on the central computer: `roslaunch robotball_localization remote_scheduler.launch`
6. Start Pozyx localization on the robot: `roslaunch robotball_localization localize.launch`
7. Start one of the controllers, e.g. `roslaunch robotball_control billiard.launch` 


## Future work
###TODO
- [ ] **Analyze and fix issues with BNO-055 IMU.** Sometimes it freezes and stops sending new data. The best guees at the moment is that it happens when the compass is close to a large metal or magnet because at the same time calibration status on all registers goes to zero.
- [ ] **Calibrate the odometry.** It didn't make sense to deal with calibration because of the frequent and large wheel slips. Once the inner structure is redesigned, it should be easier.
- [ ] **Fuse odometry data, IMU, and Pozyx for more reliable localization.**
- [ ] **Prepare a launch procedure** for everything that needs to run on the central computer. This includes common parameters used on all robots, such as default trajectory parameters.
- [ ] **Provide an interface for the simulation** that will be the same as for the real robots.
- [ ] **Analyze and fix issues in communication between Raspberry Pi and Arduino.** Maybe rosserial isn't the best solution and we could do something custom.
- [ ] **Proper cascade PID control.** Speed controller gives a pitch (acceleration) reference to the pitch controller. It controls the motors to maintain the given pitch.
- [ ] **Software support for wireless charging.** This includes self-orientation system; navigating, entering, and exiting the charger; and charging monitoring.
- [ ] Implement a single Dynamic Reconfigure for all robots when using Billiard controller.

### Bug tracking and feature requests.
If you discover bugs or missing documentation, have a question or a feature request, or something simply doesn't work, please **create a new issue** and **label** it with an appropriate label.
