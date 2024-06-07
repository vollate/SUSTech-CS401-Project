#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "record_path");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);

    std::string path_file;
    private_nh.param("path_file", path_file, std::string("/home/agilex/target_path.txt"));

    tf::TransformListener tf_listener;
    tf::StampedTransform tf_pose;

    nav_msgs::Path path_msg;

    ros::Rate r(20);
    while (ros::ok()) {
        try {
            tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.5));
            tf_listener.lookupTransform("map", "base_link", ros::Time(0), tf_pose);
        }
        catch (tf::TransformException e) {
            ROS_ERROR("%s", e.what());
        }

        static double x = tf_pose.getOrigin().getX();
        static double y = tf_pose.getOrigin().getY();

        double dx = tf_pose.getOrigin().getX() - x;
        double dy = tf_pose.getOrigin().getY() - y;
        if (sqrt(dx * dx + dy * dy) > 0.05) {
            x = tf_pose.getOrigin().getX();
            y = tf_pose.getOrigin().getY();

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "map";

            pose_msg.pose.position.x = x;
            pose_msg.pose.position.y = y;
            pose_msg.pose.position.z = 0.0;

            path_msg.header = pose_msg.header;
            path_msg.poses.push_back(pose_msg);
            path_pub.publish(path_msg);
        }

        r.sleep();
    }

    std::ofstream stream(path_file);
    for (int i = 0; i < path_msg.poses.size(); ++i) {
        stream << path_msg.poses[i].pose.position.x << " " << path_msg.poses[i].pose.position.y << std::endl;
    }

    stream.close();
    return 0;
}
