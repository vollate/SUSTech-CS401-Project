#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import *
import termios, sys, tty

class PathEditor:
    def __init__(self):
        self.path_pub = rospy.Publisher('target_path', Path, queue_size=10)
        self.server = InteractiveMarkerServer("path_editor")
        self.path = self.read_path("/home/agilex/target_path.txt")
        self.create_interactive_markers()
        self.publish_path()

    def read_path(self, file_path):
        path = Path()
        path.header.frame_id = "map"
        with open(file_path, 'r') as file:
            for line in file:
                x, y = map(float, line.split())
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                path.poses.append(pose)
        return path

    def create_interactive_markers(self):
        for i, pose in enumerate(self.path.poses):
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.pose = pose.pose
            int_marker.name = "marker_{}".format(i)
            int_marker.description = "Marker {}".format(i)

            control = InteractiveMarkerControl()
            control.always_visible = True
            control.markers.append(self.make_marker(int_marker))
            int_marker.controls.append(control)

            move_control = InteractiveMarkerControl()
            move_control.name = "move_xy"
            move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            int_marker.controls.append(move_control)

            self.server.insert(int_marker, self.process_feedback)

        self.server.applyChanges()

    def make_marker(self, msg):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def process_feedback(self, feedback):
        index = int(feedback.marker_name.split('_')[1])
        self.path.poses[index].pose = feedback.pose
        self.publish_path()

    def publish_path(self):
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def save_path(self, file_path):
        with open(file_path, 'w') as file:
            for pose in self.path.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                file.write("{} {}\n".format(x, y))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_key() == 's':
                self.save_path("/home/agilex/adjusted_path.txt")
                print("Path saved.")
            rate.sleep()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('path_editor')
    editor = PathEditor()
    try:
        editor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
