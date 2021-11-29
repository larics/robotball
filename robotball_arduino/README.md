# robotball_arduino

A "package" for all Arduino files required for driving the robot.

### Package organization
- `customLibraries` - Custom libraries commonly used in multiple sketches.
  - `LSM9_AHRS` - Attitude and Heading Reference System for LSM9DS1 IMU based on [this tutorial](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions).
  - `LSM9_Mahony` - Mahony filter for 3D orientation using LSM9DS1 IMU based on [this library](https://github.com/jremington/LSM9DS1-AHRS).
  - `Odometry` - Custom library for maintaining and reading odometry data for differentially driven robots.
  - `PID_v2` - Simplified and adapted PID library based on this [great library](https://github.com/br3ttb/Arduino-PID-Library).
  - `RobotUtilities` - A set of useful utility functions.
- `main` - Main Arduino sketch for driving the robot.
- `odom_calibration` - Arduino sketch for calibrating the odometry.

### Prerequisites - arduino-cli
The easiest way to work with Arduino from a remote computer like Raspberry Pi is to install a command-line interface to the Arduino IDE. For detailed documentation and installation instructions see [this](https://arduino.github.io/arduino-cli/0.20/).

All the steps necessary for working with arduino-cli are explained in the documentation, but the most important ones are also outlined here:
1. Create a configuration file: `arduino-cli config init`
2. Update the cache of available platforms: `arduino-cli core update-index`
3. Install the core for Nano Every: `arduino-cli core install megaavr`
4. Create auto-completion file: `arduino-cli completion bash > arduino-cli.sh` and move the created file with `sudo mv arduino-cli.sh /etc/bash_completion.d/`.
5. Install libraries:
   1. `arduino-cli lib install Adafruit_BNO055`
   2. `arduino-cli lib install arduino-timer`
   3. `arduino-cli lib install Cytron_Motor_Drivers_Library`
   4. `arduino-cli lib install Encoder`


### Prerequisites - ROS
Main Arduino sketch relies heavily on ROS. Assuming that ROS is already installed, follow these steps to create a ROS library for Arduino:
1. Install rosserial: `sudo apt-get install ros-noetic-rosserial-arduino ros-noetic-rosserial`
2. Navigate to Arduino's library directory. It is usually `cd ~/Arduino/libraries
3. Remove older ros_lib if it exists: `rm -rf ros_lib`
4. Run the script that will create a new library: `rosrun rosserial_arduino make_libraries.py .`
5. Assuming that Arduino's library is located in `~/Arduino/libraries/`, you can also use a custom alias `ard_msg` without arguments. This will work from any directory.


### Usage
To compile a sketch: `arduino-cli compile --fqbn arduino:megaavr:nona4809 --libraries ~/catkin_ws/src/robotball/robotball_arduino/customLibraries ~/catkin_ws/src/robotball/robotball_arduino/`

To upload the compiled program: `arduino-cli upload -v -p $(readlink -f /dev/tty_arduino) --fqbn arduino:megaavr:nona4809 ~/catkin_ws/src/robotball/robotball_arduino/`

For convenience, custom bash functions that replace the above commands are available for compiling and uploading the sketch:
- `ard_compile <sketch_name>`
- `ard_upload <sketch_name>`
- `ard_auto <sketch_name>` - combines two previous commands.  
These commands work from any directory.
