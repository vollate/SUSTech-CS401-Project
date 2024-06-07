#include "path.h"

WayPoint Path::findClosestPoint(const Eigen::Vector3d& pose, int start_id) {
    int max_id = std::min(start_id + 10, static_cast<int>(waypoints_.size() - 1));
    double min_dist = std::numeric_limits<double>::max();

    WayPoint closest_point;
    for (int i = start_id; i < max_id; ++i) {
        double dist = (waypoints_[i].position - pose.head<2>()).norm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_point = waypoints_[i];
        }
    }

    return closest_point;
}

WayPoint Path::findNextPoint(const WayPoint& closest_point, double distance) {
    double srch_dist = 0;

    for (int i = closest_point.id; i < waypoints_.size() - 1; ++i) {
        srch_dist += seg_dists_[i];

        if (srch_dist < distance) {
            continue;
        }
        else {
            return waypoints_[i + 1];
        }
    }

    return waypoints_.back();
}
