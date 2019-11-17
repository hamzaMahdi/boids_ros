## About
Forked from https://github.com/mkrizmancic/sphero_formation
This is a modified repository meant to explore braitenberg bahviour in flocks.
## Installation
Simply clone this repository inside ROS workspace and run `catkin_make` in workspace root.

Simulation uses _stage_ros_ simulator.

Some ros packages may be required for download (check build/run-time errors)

## Usage
1. Set values for all desired parameters inside _launch/setup_sim.launch_.
1. Set initial velocities in _cfg/sphero_init_vel.cfg_.
1. In first terminal run `roscore` (optional)
1. In second terminal run `roslaunch sphero_formation setup_sim.launch`
1. In third terminal run `roslaunch sphero_formation reynolds_sim.launch`
