# robotball_simulation

A package for simulating the robots. Mostly focused on choreography, less on dynamics and realism.


### Status
Work in progress. Simulator can be started with configuration for arbitrary number of robots and their initial positions, and offers interfaces to command the velocity and read the odometry. However, these interfaces are not the same as on the real robots.

### Usage
In [launch_params.yaml](robotballs_simulation/launch/launch_params.yaml) select the map, number of robots, and their initial distribution. Start the simulation with `rosrun robotball_simulation start.py`. 
