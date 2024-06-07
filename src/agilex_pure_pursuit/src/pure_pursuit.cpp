#include "pure_pursuit.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

void PurePursuit::backUp() {
  ros::Rate rate(10);           // 10Hz
  for (int i = 0; i < 2; ++i) { // 后退约5秒
    publishCmdVel(-0.1, 0);
    rate.sleep();
  }
  publishCmdVel(0, 0);
}

void PurePursuit::goalStatusCallback(
    const actionlib_msgs::GoalStatusArray::ConstPtr &status) {
  if (!status->status_list.empty()) {
    auto last_status = status->status_list.back().status;

    if (last_status == actionlib_msgs::GoalStatus::ABORTED) {
      ROS_WARN("Navigation aborted! Attempting to free robot by backing up.");
      backUp();
      move_base_client_.cancelAllGoals();
      ROS_INFO("All move_base goals cancelled after backup.");
      // 可以在此尝试再次发送目标或者进行其他处理
    }
  }
}

// double PurePursuit::calculateSteeringAngle(const WayPoint& p) {
//     double lookahead_distance = 1.0; // This should be dynamically adjusted
//     based on speed or other factors

//     // Calculate the angle between the robot's current pose and the target
//     point double angle_to_target = atan2(p.position(1) - cur_pose_(1),
//     p.position(0) - cur_pose_(0)); double heading_error =
//     normalizeAngle(angle_to_target - cur_pose_(2));

//     // Calculate steering angle using the bicycle model
//     // steering_angle = atan((2 * wheelbase * sin(heading_error)) /
//     lookahead_distance); double steering_angle = atan2(2 * wheelbase_ *
//     sin(heading_error), lookahead_distance);

//     // Limit the steering angle to vehicle's max steering angle
//     double max_steering_angle = M_PI / 6; // Example: max 30 degrees steering
//     angle steering_angle = std::max(std::min(steering_angle,
//     max_steering_angle), -max_steering_angle);

//     return steering_angle;
// }

PurePursuit::PurePursuit() : move_base_client_("move_base", true) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ROS_INFO("pure pursuit constructor called");
  // 在PurePursuit类中添加成员变量
  stop_for_cross_ = false;
  slow_for_signal_ = false;
  traffic_light_state_ = 0; // 0: green, 2: yellow, 1: red

  std::string scan_topic, pose_topic, cmd_vel_topic, path_topic;
  private_nh.param("scan_topic", scan_topic, std::string("scan"));
  private_nh.param("pose_topic", pose_topic, std::string("pose"));

  private_nh.param("path_topic", path_topic, std::string("target_path"));
  private_nh.param("cmd_vel_topic", cmd_vel_topic, std::string("cmd_vel"));

  private_nh.param("obstacle_check_x_range", obstacle_check_x_range_, 0.5);
  private_nh.param("obstacle_check_y_range", obstacle_check_y_range_, 0.3);

  private_nh.param("position_control_tolerance", position_control_tolerance_,
                   0.01);
  private_nh.param("angle_control_tolerance", angle_control_tolerance_, 0.3);

  private_nh.param("max_lin_vel", max_lin_vel_, 0.5);
  private_nh.param("max_ang_vel", max_ang_vel_, M_PI);

  private_nh.param("position_kp", position_kp_, 1.0);
  private_nh.param("angle_kp", angle_kp_, 5.0);

  private_nh.param("look_ahead_dist", look_ahead_dist_, 1.0);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  // cmd_vel_pub_ =
  // nh.advertise<ackermann_msgs/AckermannDriveStamped>(cmd_vel_topic, 1);
  next_waypoint_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("next_waypoint", 1);
  target_path_pub_ = nh.advertise<nav_msgs::Path>("target_path", 1, true);
  goal_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  status_sub = nh.subscribe("move_base/status", 1,
                            &PurePursuit::goalStatusCallback, this);

  scan_sub_ = nh.subscribe(scan_topic, 1, &PurePursuit::scanCallback, this);

  cross_sub_ = nh.subscribe("detector/cross/result", 1,
                            &PurePursuit::crossCallback, this);
  signal_sub_ = nh.subscribe("detector/signal/result", 1,
                             &PurePursuit::signalCallback, this);
  traffic_light_sub_ = nh.subscribe("detector/light/result", 1,
                                    &PurePursuit::trafficLightCallback, this);

  nav_msgs::Path path_msg;
  if (loadTargetPath("/home/agilex/target_path.txt", path_msg)) {
    makePath(path_msg);
    target_path_pub_.publish(path_msg);
    state_ = START;
  } else {
    ROS_ERROR("Failed to load path file!");
  }
}

void PurePursuit::crossCallback(const std_msgs::Int16 &msg) {
  if (!sleep_for_cross_ && msg.data == 1) {
    stop_for_cross_ = true;
    sleep_for_cross_ = true;
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      sleep_for_cross_ = false;
      ROS_INFO("Sleep complete, ready for next crossing signal.");
    }).join();

  } else {
    stop_for_cross_ = false;
  }
}

void PurePursuit::signalCallback(const std_msgs::Int16 &msg) {
  slow_for_signal_ = (msg.data == 1);
}

void PurePursuit::trafficLightCallback(const std_msgs::Int16 &msg) {
  traffic_light_state_ = msg.data;
}

void PurePursuit::publishNewGoal(const WayPoint &new_target) {
  geometry_msgs::PoseStamped new_goal;
  new_goal.header.stamp = ros::Time::now();
  new_goal.header.frame_id = "map"; // 确保使用正确的坐标系
  new_goal.pose.position.x = new_target.position(0);
  new_goal.pose.position.y = new_target.position(1);
  new_goal.pose.orientation =
      tf::createQuaternionMsgFromYaw(new_target.heading); // 设置正确的朝向

  ROS_INFO("publishing [%f, %f]", new_target.position(0),
           new_target.position(1));
  goal_pub_.publish(new_goal);
}

bool PurePursuit::loadTargetPath(const std::string &path_file,
                                 nav_msgs::Path &path_msg) {
  std::ifstream stream(path_file);
  if (!stream) {
    ROS_ERROR("could not open %s", path_file.c_str());
    return false;
  }

  std::string str;
  while (std::getline(stream, str)) {
    std::stringstream ss(str);
    double x, y;
    ss >> x >> y;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    path_msg.header = pose_msg.header;
    path_msg.poses.push_back(pose_msg);
  }

  return true;
}

double PurePursuit::normalizeAngle(double angle) {
  while (angle >= M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

bool PurePursuit::getPose() {
  tf::StampedTransform tf;
  try {
    tf_listener_.waitForTransform("map", "base_link", ros::Time(0),
                                  ros::Duration(1.0));
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), tf);
  } catch (tf::TransformException e) {
    ROS_ERROR("%s", e.what());
    return false;
  }

  geometry_msgs::TransformStamped tf_msg;
  tf::transformStampedTFToMsg(tf, tf_msg);
  cur_pose_(0) = tf_msg.transform.translation.x;
  cur_pose_(1) = tf_msg.transform.translation.y;
  cur_pose_(2) = tf::getYaw(tf_msg.transform.rotation);

  return true;
}

void PurePursuit::detectOobstacle(const sensor_msgs::LaserScan &scan) {
  obstacle_distance_ = std::numeric_limits<double>::max();
  int ranges = scan.ranges.size();
  for (int i = 0; i < ranges; ++i) {
    if (scan.ranges[i] < scan.range_min)
      continue;

    double angle = scan.angle_min + i * scan.angle_increment;
    double x = scan.ranges[i] * cos(angle);
    double y = scan.ranges[i] * sin(angle);

    if (x > 0 && fabs(y) < obstacle_check_y_range_) {
      // x += 0.2*fabs(y);
      if (x < obstacle_distance_) {
        obstacle_distance_ = x;
      }
    }
  }
}

bool PurePursuit::checkHeading(const WayPoint &p) {
  double error = normalizeAngle(p.heading - cur_pose_(2));
  static int check_cnt = 0;

  if (fabs(error) < angle_control_tolerance_) {
    check_cnt++;
    if (check_cnt > 10) {
      check_cnt = 0;
      return true;
    } else {
      return false;
    }
  } else {
    check_cnt = 0;
    return false;
  }
}

bool PurePursuit::checkPosition(const WayPoint &p) {
  double error = cos(cur_pose_(2)) * (p.position(0) - cur_pose_(0)) +
                 sin(cur_pose_(2)) * (p.position(1) - cur_pose_(1));
  static int check_cnt = 0;

  if (fabs(error) < position_control_tolerance_) {
    check_cnt++;
    if (check_cnt > 10) {
      check_cnt = 0;
      return true;
    } else {
      return false;
    }
  } else {
    check_cnt = 0;
    return false;
  }
}

double PurePursuit::calculateAngVel(const WayPoint &p) {
  double error = normalizeAngle(p.heading - cur_pose_(2));
  double ang_vel = error * angle_kp_;
  if (ang_vel > 0) {
    ang_vel = std::min(ang_vel, max_ang_vel_);
  } else {
    ang_vel = std::max(ang_vel, -max_ang_vel_);
  }

  return ang_vel;
}

// void PurePursuit::backUpAndRotate() {
//     // 发送后退命令
//     geometry_msgs::Twist back_up_msg;
//     back_up_msg.linear.x = -0.1;  // 后退速度
//     cmd_vel_pub_.publish(back_up_msg);

//     // 等待一小段时间后退
//     ros::Duration(1.0).sleep();

//     // 停止后退
//     back_up_msg.linear.x = 0;
//     cmd_vel_pub_.publish(back_up_msg);

//     // 发送旋转命令
//     geometry_msgs::Twist rotate_msg;
//     rotate_msg.angular.z = 0.5; // 设置适当的旋转速度
//     cmd_vel_pub_.publish(rotate_msg);

//     // 旋转一定时间
//     ros::Duration(2.0).sleep();

//     // 停止旋转
//     rotate_msg.angular.z = 0;
//     cmd_vel_pub_.publish(rotate_msg);
// }

void PurePursuit::calculateVel(const WayPoint &closest_p,
                               const WayPoint &next_p, double &lin_vel,
                               double &ang_vel) {
  double dx = cos(cur_pose_(2)) * (next_p.position(0) - cur_pose_(0)) +
              sin(cur_pose_(2)) * (next_p.position(1) - cur_pose_(1));
  double dy = -sin(cur_pose_(2)) * (next_p.position(0) - cur_pose_(0)) +
              cos(cur_pose_(2)) * (next_p.position(1) - cur_pose_(1));
  double ds = sqrt(dx * dx + dy * dy);

  if (obstacle_distance_ < dx && obstacle_distance_ < obstacle_check_x_range_) {
    ROS_WARN_THROTTLE(1.0, "Obstacle detected! Calculating new target point.");
    // WayPoint new_target = selectNewTargetPoint(next_p);
    WayPoint new_target = path_.findNextPoint(closest_p, 1.2);
    publishNewGoal(new_target); // 发布新目标点
    return;
  } else {
    move_base_client_.cancelAllGoals();
  }

  lin_vel = max_lin_vel_;
  if (fabs(dy) < 1e-3) {
    ang_vel = 0.0;
  } else {
    double rho = ds * ds / (2 * dy);
    ang_vel = lin_vel / rho;
  }

  if (fabs(normalizeAngle(closest_p.heading - next_p.heading)) < 1e-3) {
    ang_vel = angle_kp_ * ang_vel;
  }

  if (ang_vel > 0) {
    ang_vel = std::min(ang_vel, max_ang_vel_);
  } else {
    ang_vel = std::max(ang_vel, -max_ang_vel_);
  }
}

void PurePursuit::makePath(const nav_msgs::Path &path_msg) {
  path_.clear();

  int n = path_msg.poses.size();
  for (int i = 0; i < n - 1; ++i) {
    Eigen::Vector2d position(path_msg.poses[i].pose.position.x,
                             path_msg.poses[i].pose.position.y);

    double dx = path_msg.poses[i + 1].pose.position.x - position(0);
    double dy = path_msg.poses[i + 1].pose.position.y - position(1);
    double theta = atan2(dy, dx);

    WayPoint p(position, theta);
    path_.push_back(p);
  }

  Eigen::Vector2d position(path_msg.poses[n - 1].pose.position.x,
                           path_msg.poses[n - 1].pose.position.y);
  double theta = path_.getEndPoint().heading;
  WayPoint p(position, theta);
  path_.push_back(p);
}

void PurePursuit::scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg) {
  if (state_ == IDLE)
    return;
  detectOobstacle(*scan_msg);
}

void PurePursuit::track() {
  double lin_vel = 0, ang_vel = 0;

  if (!getPose())
    return;

  // 进行常规速度计算
  static WayPoint closest_waypoint, next_waypoint;

  switch (state_) {
  case IDLE: {
    break;
  }
  case START: {
    WayPoint start_point = path_.getStartPoint();
    if (checkHeading(start_point)) {
      closest_waypoint = path_.getStartPoint();
      next_waypoint = path_.findNextPoint(closest_waypoint, look_ahead_dist_);
      state_ = TRACK;
    } else {
      ang_vel = calculateAngVel(start_point);
    }
    break;
  }
  case TRACK: {
    if (next_waypoint.id == path_.getEndPoint().id) {
      closest_waypoint = path_.getStartPoint();
      next_waypoint = path_.getStartPoint();
      calculateVel(closest_waypoint, next_waypoint, lin_vel, ang_vel);
    } else {
      closest_waypoint = path_.findClosestPoint(cur_pose_, closest_waypoint.id);
      next_waypoint = path_.findNextPoint(closest_waypoint, look_ahead_dist_);
      calculateVel(closest_waypoint, next_waypoint, lin_vel, ang_vel);
    }
    break;
  }
  }

  switch (traffic_light_state_) {
  case 2: // Red Light
    lin_vel = 0;
    ROS_INFO("Red light detected - stopping.");
    break;

  case 1:           // Yellow Light
    lin_vel *= 0.1; // Reduce speed to 10%
    ang_vel *= 0.1;
    ROS_INFO("Yellow light detected - reducing speed.");
    break;

  default:
    // No action needed, lin_vel remains at max_lin_vel_
    break;
  }

  if (slow_for_signal_) {
    lin_vel *= 0.1; // Reduce speed to 10%
    ang_vel *= 0.1;
    ROS_INFO("Signal detected - reducing speed.");
  }
  // Handle pedestrian crossing
  if (stop_for_cross_) {
    lin_vel = 0;
    ROS_INFO("Cross detected - stopping.");
    ros::Duration(1.0).sleep();
  }

  publishCmdVel(lin_vel, ang_vel);
  publishNextWayPoint(next_waypoint);
}

void PurePursuit::publishNextWayPoint(const WayPoint &p) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "map";
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, p.heading);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.position.x = p.position(0);
  pose_msg.pose.position.y = p.position(1);
  pose_msg.pose.position.z = 0.0;

  next_waypoint_pub_.publish(pose_msg);
}

void PurePursuit::publishCmdVel(double lin_vel, double ang_vel) {
  geometry_msgs::Twist vel_msg;

  vel_msg.linear.x = lin_vel;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = ang_vel;

  cmd_vel_pub_.publish(vel_msg);
}

void PurePursuit::run() {
  ros::Rate rate(50);

  while (ros::ok()) {
    track();
    rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  PurePursuit p;
  p.run();

  return 0;
}
