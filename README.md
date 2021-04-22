# EG2310 Group 8 Autonomous Navigation Turtlebot With IR Detecting Ping-Pong Cannon

Group 8 has modified the turtlebot to perform autonomous navigation and given an IR target, it will cease autonomous navigation and fire at the target. It will then continue its autonomous navigation.

# Required OS and Packages
1) Ubuntu 20.04.2.0 LTS Desktop (64 bit)
2) ROS 2 Foxy Fitzroy
3) Turtlebot Packages
4) Ubuntu 20.04.1(Focal) Preinstalled Server (for Raspberry Pi)
5) OpenSSH
6) OpenCR Firmware
7) Python IDE

# Installation
The full set-up from scratch is provided in the wiki. We assume that you already have the relevant packages as stated above.

# Overview
Software flow for mapping and firing.

<p align="center">
	<img src="doc/soft_flow.png" width="750"/>
</p>

# Navigation
For occupancy grid data, -1 value is mapped to 1, and occupancy grid values ranging from 0 to 60 are mapped to 2 and 60 to 100 are mapped to 3.

In this case, 1 is unmapped, 2 is free space, and 3 is blocked.

Map file is also saved under 'map.txt'.

For navigation, Breadth First search is used to find unmapped areas. Breadth first search starts from robot's current position.

Once an unmapped area has been found, A* Search is used to find path between robot's current position and the unmapped area.

Turtlebot follows the global path by updating its' path on the fly. It always attempts to go to 1st item in the path, and pops the item if it has reached the coordinate. It is like a pacman following a line.

For visualization purposes, if path is found, it is published to /global_plan topic. One can use Rviz to visualize the path.

## Future Improvements
* Global Path Smoothing
* Uses control signal to follow the global path
* Implements obstacle avoidance (Local Planner)
	
	


