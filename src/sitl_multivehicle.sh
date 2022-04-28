#!/bin/bash
gnome-terminal --tab --title="Arducopter MavProxy" -- sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --console -I0
gnome-terminal --tab --title="APMrover2 MavProxy" -- sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --console -I1