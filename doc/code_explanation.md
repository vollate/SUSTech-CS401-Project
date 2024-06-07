# Code Explaination

## RRT Exploration

In pkg `rrt_exploration`.

The RRT algorithm has these steps:

- Setup and Initialization: The robots and environment are set up in ROS, initializing the necessary nodes for mapping and exploration.
- Frontier Detection: Using image processing techniques, the algorithm detects frontiers in the map. Frontiers are unexplored boundaries between known free space and unknown space.
- RRT-based Exploration: The algorithm uses the RRT approach to explore the detected frontiers. RRT efficiently expands the search space by generating random points and connecting them to the nearest existing point in the tree, thus covering the map rapidly.
- Multi-robot Coordination: In a multi-robot setup, the exploration tasks are distributed among the robots. Each robot receives a target frontier to explore, ensuring efficient coverage of the environment.
- Navigation and Mapping: The robots navigate towards the assigned frontiers, update the map with new information, and repeat the process until the entire area is explored.
- Visualization and Customization: The package supports visualization in Rviz, allowing users to monitor the exploration process. Various parameters can be customized to optimize the algorithm for specific use cases.

However, for orginal RRT package, it have many bugs.

Firstly, due to the very old version of GNU compiler on limo, the function signature have error. On ARM, the default `char` means `unsigned char`, make the return value of -1 actually 255. This is the shit of C/C++ history standard, which tells us always use fixed width integer types for program that may run on different architectures or platforms.

Secondly, the `rrt` package's function `makePlan` is not call at anywhere of its code, which means it could not send any plan request to the planner. We fix this by set it target to "move_base/make_plan" service. Also, we need to make sure when the plan request is sent, the move_base don't have any goal to go. Otherwise, the move_base will crash.

Due to the poor performance of limo's hardware, we have to decrease the rate of updating for global and local costmap, and also limited the robot's speed to avoid collision for the hight latency of the whole system.

## Line tracking with signal detection

### Pure Pursuit

In pkg `agilex_pure_pursit`.

#### Handling Detector Signals

Traffic signals are handled by the `signalCallback` and `trafficLightCallback` functions, with subscriptions to the relevant topics:

- **signal_sub**: Subscribes to the topic `detector/signal/result` and triggers the `signalCallback` function when a signal message is received. The `signalCallback` function checks if the message data (`msg.data`) equals 1, setting the `slow_for_signal_` flag to true. This flag is then used in the `track` method to reduce the robot's speed to 10% of the maximum when a signal is detected.

- **traffic_light_sub**: Subscribes to the topic `detector/light/result` and triggers the `trafficLightCallback` function when a traffic light message is received. The state of the traffic light (`msg.data`) is stored in the `traffic_light_state_` variable. In the `track` method, the robot's behavior is adjusted based on this state:
  - Red Light (state 2): The robot stops by setting the linear velocity to 0.
  - Yellow Light (state 1): The robot slows down by reducing both linear and angular velocities to 10% of their maximum values.
  - Green Light (default state): The robot continues at normal speed.

- **cross_sub**: Subscribes to the topic `detector/cross/result` and triggers the `crossCallback` function when a crosswalk message is received. The `crossCallback` function sets the `stop_for_cross_` flag to true if the message data (`msg.data`) equals 1 and ensures the robot stops at the crosswalk by setting the linear velocity to 0 and invoking a brief sleep period.

#### Finding the Next Target

The next target point on the path is determined using methods defined in the `path.h` file:

- **findClosestPoint**: This method finds the closest waypoint to the robot's current position. It starts searching from a given waypoint ID (`start_id`) and looks for the waypoint with the minimum distance to the current position. It only searches within a specified range to limit the computational load.

- **findNextPoint**: This method identifies the next waypoint that the robot should aim for, based on a given distance from the closest point. It iterates through the waypoints starting from the closest point and accumulates the distance until the specified look-ahead distance is reached. The waypoint at this distance is chosen as the next target point, ensuring a smooth and continuous trajectory.

These methods allow the Pure Pursuit algorithm to dynamically update the robot's target as it moves along the path, ensuring accurate path tracking and efficient navigation.

### Signal Detection

In pkg `detector_node`.

The `detector_node` code is designed to detect traffic signals, crosswalks, and traffic lights using image processing techniques within a ROS-based system. It uses the OpenCV library for image processing and ROS for communication. Here's a detailed explanation:

#### Detector Class

The `Detector` class handles the image processing and publishing of detection results.

**Initialization:**

- The constructor initializes the ROS publishers for various topics, including:
  - Binary images and contours for traffic signals, crosswalks, and traffic lights.
  - Detection results for signals, crosswalks, and lights as `std_msgs::Int16` messages.

**Image Processing Methods:**

- `compressImg`: This method resizes the input image by a specified ratio to reduce computational load.
- `binary`: Converts the input image to grayscale and applies a binary threshold to highlight areas of interest.
- `red_binary` and `yellow_binary`: These methods specifically extract and threshold the red and yellow components of the image, respectively. They help in isolating red and yellow traffic lights from the rest of the image.

**Detection Methods:**

- `detect_signal`: Detects traffic signals in the image. It processes the image to find contours, filters them based on area and shape, and checks if the contours resemble a circular shape. If a valid contour is found, it publishes the binary image and contours and returns 1 (indicating a signal is detected); otherwise, it returns 0.
- `detect_traffic_binary`: A helper method used in traffic light detection. It finds and validates contours based on area, shape, and brightness of the center.
- `detect_traffic_lights`: Uses the helper method to detect red and yellow traffic lights. It processes the image to find and validate contours, then publishes the binary images and contours. It returns 2 if a red light is detected, 1 for a yellow light, and 0 for no light.
- `detect_cross`: Detects crosswalks in the image. It processes the image to find contours, filters them based on area and rectangular shape, and checks their position. If valid contours are found, it publishes the binary image and contours and returns 1; otherwise, it returns 0.

#### Publish Topics

- **signal_sub** and **cross_sub**: The node subscribes to these topics to receive messages about traffic signals and crosswalks, respectively. These messages are processed in the callback functions (`signalCallback` and `crossCallback`) to adjust the robot's behavior based on the detected signals.

- **signal_result_pub**, **cross_result_pub**, **light_result_pub**: These publishers send the detection results as integer messages, which can be used by other nodes to make decisions based on the detected traffic signals, lights, and crosswalks.

This code effectively integrates image processing and ROS communication to detect important features in the environment and publish the results for use in a robotic system.
