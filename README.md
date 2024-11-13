# # Assignment 1 RT

This package contains two nodes:
- `ui_node`: Spawns a new turtle (turtle2) and provides a simple UI for controlling turtle1 or turtle2.
- `distance_node`: Monitors the distance between turtle1 and turtle2, and stops the turtles if they are too close or near boundaries.

## How to Run

1. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
