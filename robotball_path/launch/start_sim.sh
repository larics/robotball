#!/bin/bash

number_of_nodes=$(rosparam get /num_of_robots)
echo "Launching $number_of_nodes robots..."

trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}

for ((i=0; i<$number_of_nodes; i++));
do
	ROS_NAMESPACE="robot_$i" rosrun robotball_path reference_follower.py &
	ROS_NAMESPACE="robot_$i" rosrun robotball_path reference_generator.py &
done
echo "DONE"

cat
