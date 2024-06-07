# How To Run

## RRT Exploration

```sh
roslaunch sentry_bringup rrt.launch
```

Then, one the started rviz, click five points on the map. The first four points set the exploration boundary, and the last the tree root of the RRT.

## Line tracking with signal detection

```sh
roslaunch agilex_pure_pursuit pure_pursuit.launch
roslauch sentry_bringup detect.launch
```

To make robot locate itself with high accuracy, you can use the command <TODO> to adjust the lidar's point cloud. The more aligned the point cloud is with the map, the more accurate the robot's location will be.