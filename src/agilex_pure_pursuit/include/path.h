#ifndef PATH_H
#define PATH_H

#include <vector>
#include <Eigen/Core>

struct WayPoint {
    WayPoint() {}
    WayPoint(const Eigen::Vector2d& position, double heading) {
        this->position = position;
        this->heading = heading;
    }

    int id;
    Eigen::Vector2d position;
    double heading;
};

class Path {
public:
    Path() {}
    ~Path() {}
    void clear() {
        waypoints_.clear();
        seg_dists_.clear();
    }
    int size() { return waypoints_.size(); }
    void push_back(const WayPoint& p) {
        if (waypoints_.size() > 0) {
            double seg_dist = (waypoints_.back().position - p.position).norm();
            seg_dists_.push_back(seg_dist);
        }
        waypoints_.push_back(p);
        waypoints_.back().id = waypoints_.size() - 1;
    }

    WayPoint getStartPoint() { return waypoints_[0]; }
    WayPoint getEndPoint() { return waypoints_.back(); }

    WayPoint findClosestPoint(const Eigen::Vector3d& pose, int start_id);
    WayPoint findNextPoint(const WayPoint& closest_point, double distance);
private:
    std::vector<WayPoint> waypoints_;
    std::vector<double> seg_dists_;
};

#endif // PATH_H
