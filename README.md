## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
`motion_planning.py` contains planning class MotionPlanning which inherits from a Udacity-provided Drone class, for methods provided by this class check this API: https://udacity.github.io/udacidrone/docs/drone-api.html 
This is script file uses principles of event driven programming.
1. At the start this script connects to Udacity simulator using Mavlink (you can download the simulator here https://github.com/udacity/FCND-Simulator-Releases/releases)
2. Starts the drone using Arming and Manual
3. in method plan_path() is computed path for navigation
4. contruct a grid map from 2.5D map in colliders.csv file 
5. contruct path using A* star algorith from grid map
6. sending waypoint constructed from path

`planning_utils.py` contains utilities for
 1. creating a grid map
 2. defines valid actions fro drone to take
 3. implementation of A* star algorithm (grid implementation) with its heuristic function
 4. create_grid function for grid map in desired altitude
 5. prune_path function which uses collinearity check to prune the path

And here's a lovely image of the drone executing the computed navigation path.
![Top Down View](./misc/executing_path.png)

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I have read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home.
For reading the coordinates I am using splitting strings to parse the first line of the csv file
```python
filename = 'colliders.csv'
with open(filename) as f:
    origin_pos_data = f.readline().split(',')
lat0 = float(origin_pos_data[0].strip().split(' ')[1])
lon0 = float(origin_pos_data[1].strip().split(' ')[1])

self.set_home_position(lon0, lat0, 0)
```

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/grid.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set.

To set current position I firstly need to get a global position of the drone. Then I have to convert this global position to local position.

```python
 # retrieve current global position
current_global_position = [self._longitude, self._latitude, self._altitude]

# convert to current local position using global_to_local()
current_local_position = global_to_local(current_global_position, self.global_home)
```

#### 3. Set grid start position from local position
To convert to grid positions we first need to obtain north_offset, east_offset from create_grid function. Then we set the start position as follows
```python
grid_start = (int(np.round(current_local_position[0])) - north_offset, int(np.round(current_local_position[1])) - east_offset)
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

To generate grid goal from geodetic coords I am using this line of code:
```python
goal_lon = -122.399219
goal_lat = 37.794262
goal_global = (goal_lon, goal_lat, 0)
goal_local = global_to_local(goal_global, self.global_home)
grid_goal = (int(np.round(goal_local[0])) - north_offset, int(np.round(goal_local[1])) - east_offset)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

I have to add more allowed actions:
```python
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```

And for each of them add a check if it this action will not end up outside the map:
```python
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```

#### 6. Cull waypoints 
Final step was to prune the computed path from A* search to make the plan a flight of the drone more smoothly. To prune the path I am using collinearity test.
```python
m = np.concatenate((p1, p2, p3), 0)
det = np.linalg.det(m)
return abs(det) < epsilon 
```

### Execute the flight
#### 1. Does it work?
The drone is following the path. But there is some space for improvement, like more clever bounds to make the drone move more smoothly.
