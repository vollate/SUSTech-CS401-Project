#ifndef functions_H
#define functions_H
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

// rdm class, for gentaring random flot numbers
class rdm {
  int i;

public:
  rdm();
  float randomize();
};

// Norm function prototype
float Norm(std::vector<float>, std::vector<float>);

// sign function prototype
float sign(float);

// Nearest function prototype
std::vector<float> Nearest(std::vector<std::vector<float>>, std::vector<float>);

// Steer function prototype
std::vector<float> Steer(std::vector<float>, std::vector<float>, float);

// gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &, std::vector<float>);

// ObstacleFree function prototype
signed char ObstacleFree(std::vector<float>, std::vector<float> &,
                         nav_msgs::OccupancyGrid);
#endif
