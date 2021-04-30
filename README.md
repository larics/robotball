# robotball

Packages for robotic balls developed by LARICS and TU Delft.

## Arduino
Arduino sketches are located [here](robotball_arduino). Name of the directory corresponds to the name of the main .ino file.  
Take main.ino for example...  
`cd <path_to_your_workspace>/src/robotball/robotball_arduino`

Building the code: `arduino-cli compile --fqbn arduino:avr:nano main`  
Uploading the code: `arduino-cli upload -v -t -p /dev/ttyACM0 --fqbn arduino:avr:nano main`  
Searching for libraries:
```
arduino-cli lib update-index
arduino-cli lib search <library name>
```
Installing libraries: `arduino-cli lib install "<exact library name"`  
Uninstalling libraries: `arduino-cli lib uninstall "<exact library name"`  
(Un)installing specific version of a library: `arduino-cli lib (un)install "<exact library name>"@1.2.3`  


## ROS
Control the ball with the keyboard: `roslaunch robotball_driver keyboard.launch`  
Control the ball with a joystick: `roslaunch robotball_driver joystick.launch joy_config:=f710`  
Control the ball with a joystick connected to remote PC:
- Check remote master settings.
- on the rPi: `roslaunch robotball_driver bare.launch`
- on the PC: `roslaunch robotball_driver only_joystick.launch joy_config:=f710`

More information to come...
