# Update Plan

## New UI features:
1. View active nodes and their topics in the UI
2. View gps status (receiving correct ID, current gps position readings, etc)
3. RPLidar point cloud


## New path planner features:
4. Add a new Arial Potential Field (APF) path planner in `robot/path_planner.py`; we will avoid using current `PurePursuitPlannerWithAvoidance` since it is not built with good quality. But we will keep the support in the codebase since it's released already. The new path planner can work with current pure pursuit planner, and it has the same handle, blocking/non-blocking, etc.

For 1, 2, and 3, we will add them in the current rpi sensor panels in the UI. 
For 2, and 3, we can combine them into one canvas. The canvas has (0,0) at bottom left. The coordinate is x-axis to right, y to top. Will draw to robot position in the canvas, we can then plot rplidar point cloud in it too, which will need to consider the robot position and lidar installation position and orientation. Curernt main,py does handle this but it's really poorly implemented. We will still keep it, but writing some new code. We can add a lidar_scan.py that can be imported in main.py. It can be configured for rplidar installation, process the raw lidar scan data, and then return the point cloud. The new path planner can then use the point cloud for obstacle avoidance.

Design the new code structure cleanly.

For some new information we want to show in the UI, we can add a new topic for gps status and point cloud. Help me design the message type for these topics.

## Some important rules:
0. Do not touch main.py at all; it's released already. We will create a new example code and I will paste it in main manually. 
1. Do not start implementation before getting a clear instruction. 
2. We will also discuss more details first too.
3. Do not commit if the code is not fully tested.
4. Make the commit message concise and clear. Do not add any author, co-author information.

