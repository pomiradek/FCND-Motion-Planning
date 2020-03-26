# FCND-Motion-Planning
Udacity Flying Car Nanodegree Motion Planning project

## Starter Code

### motion_planning.py
This script file contains planning class MotionPlanning which inherits from a Udacity-provided Drone class, for methods provided by this class check this API: https://udacity.github.io/udacidrone/docs/drone-api.html 
This is script file uses principles of event driven programming.
1. At the start this script connects to Udacity simulator using Mavlink (you can download the simulator here https://github.com/udacity/FCND-Simulator-Releases/releases)
2. Starts the drone using Arming and Manual
3. in method plan_path() is computed path for navigation
4. contruct a grid map from 2.5D map in colliders.csv file 
5. contruct path using A* star algorith from grid map
6. sending waypoint constructed from path
