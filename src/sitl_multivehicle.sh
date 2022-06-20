#!/bin/bash
gnome-terminal --tab --title="Arducopter MavProxy" -- sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
gnome-terminal --tab --title="APMrover2 MavProxy" -- sim_vehicle.py -v APMrover2 --console -I1