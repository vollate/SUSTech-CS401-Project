# Code Explaination

## RRT Exploration

For orginal RRT package, it have many bugs.

Firstly, due to the very old version of compiler on limo(gcc 7), the function signature have strange error that its `char` means `unsigned char`, make the return value of "-1" actually 255.

Secondly, the `rrt` package's function `makePlan` is not call at anywhere of its code, which means it could not send any plan request to the move_base planner. We fix this by set it target to "move_base/make_plan" service.

Due to the poor performance of limo's hardware, we have to decrease the rate of updating for global and local costmap, and also limited the robot's speed to avoid collision for the hight latency of the whole system.

## Line tracking with signal detection

### Pure Pursuit

//TODO

### Signal Detection

@Mango
