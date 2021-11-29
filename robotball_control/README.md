# robotball_control

A package for various high-level controllers: position PID, Lissajous curve tracker, geometric shape tracker, etc.

## Usage
### Open-loop contoller
Open-loop controller consists of just one script that continously sends predefined velocity commands without any regard for robot's position or speed. Change the velocity profile directly in the script.

To run it, use `roslaunch robotball_control open_loop.launch`.

### Lissajous curve tracking
This contoller consists of a trajectory generator and a trajectory follower. Desired properties of Lissajous curves (shape, size, location) can be dynamically changed using Dynamic Reconfigure. Default parameters are currently loaded for all robots, on all robots. They can be found in [cfg/lissajous](robotball_control/cfg/lissajous).

Trajectory generator continuously updates the position reference for position PID located in the follower node. PID parameters can also be dynamically adjusted.

To run it, specify the number of robots, type, and default setup in [generators.launch](robotball_control/launch/generators.launch) and then use `roslaunch robotball_control generators.launch`.

### Geometric curve tracking
This contoller consists of a trajectory generator and a trajectory follower. Desired properties of geometric shapes (number of points, size, rotation) can be dynamically changed using Dynamic Reconfigure. Default parameters are currently loaded for all robots, on all robots. They can be found in [cfg/geometric](robotball_control/cfg/geometric). 

Trajectory generator continuously updates the position reference for position PID located in the follower node. PID parameters can also be dynamically adjusted.

To run it, specify the number of robots, type, and default setup in [generators.launch](robotball_control/launch/generators.launch) and then use `roslaunch robotball_control generators.launch`.

### Billiard controller
This controller doesn't use a reference. When initialized, a random direction and a constant speed are commanded to the robot. Upon reaching the edge of the arena (virtual wall), the direction is changed to "bounce" the robot back, just like a billiard ball. To prevent collisions, robots feel a repulsive force between them. In the case that the robots spends too much time outside the specified area, direction is changed to point towards the middle.

Size of the arena, bounderies from which robots bounce back, and names of the robots used need to be specified in [default_billiard.yaml](robotball_control/cfg/default_billiard.yaml). Currently, these parameters are loaded for all robots, on all robots. Robot's speed, allowed duration outside the area, separation from other robots, and strength of the repulsion force can be dynamically changed.

To run it, use `roslaunch robotball_control billiard.launch`.
