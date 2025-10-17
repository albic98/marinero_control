#!/usr/bin/env python3

import math
import rclpy
import rclpy.time
from tf2_ros import Buffer, TransformListener, LookupTransform
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry

class MarineroMarker(Node):
    def __init__(self):
        super().__init__("marinero_marker")
        self.odom_sub = self.create_subscription(Odometry, "/marinero/odom", self.location_callback, 10)
        self.marinero_publisher = self.create_publisher(MarkerArray, "/marinero_tracker", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.position = (0.0, 0.0)
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Reuse markers by creating them once
        self.marker_line = self.create_marker_line(Point(), Point())
        self.marker_sphere = self.create_marker_sphere(Point())
        self.marker_arrow = self.create_marker_arrow(Point())
        self.camera_marker_arrow = self.create_camera_marker_arrow(Point())

        # Counter for throttling publishing
        self.counter = 0
        
        self.create_timer(0.05, self.publish_markers)


    def location_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.height = msg.pose.pose.position.z
        self.orientation = msg.pose.pose.orientation

        self.pose = PoseStamped()
        self.pose.header.frame_id = "base_link"
        self.pose.pose.position.x = self.position[0]
        self.pose.pose.position.y = self.position[1]
        self.pose.pose.position.z = self.height
        self.pose.pose.orientation = self.orientation

        # self.publish_markers() # Publish markers for visualization in RViz

    def create_marker_line(self, point1, point2):
        marker_line = Marker()
        marker_line.header.frame_id = "odom"
        marker_line.type = Marker.CYLINDER
        marker_line.action = Marker.ADD
        marker_line.scale.x = 0.5
        marker_line.scale.y = 0.5
        marker_line.scale.z = 15.0
        marker_line.color.a = 1.0
        marker_line.color.r = 1.0
        marker_line.color.g = 0.0
        marker_line.color.b = 0.0
        marker_line.ns = "connected_markers"
        marker_line.id = 0
        return marker_line

    def create_marker_sphere(self, point):
        marker_sphere = Marker()
        marker_sphere.header.frame_id = "odom"
        marker_sphere.type = Marker.SPHERE
        marker_sphere.action = Marker.ADD
        # marker_sphere.pose.position = point
        # marker_sphere.pose.orientation.w = 1.0
        marker_sphere.scale.x = 5.0
        marker_sphere.scale.y = 5.0
        marker_sphere.scale.z = 5.0
        marker_sphere.color.a = 1.0
        marker_sphere.color.r = 1.0
        marker_sphere.color.g = 0.0
        marker_sphere.color.b = 0.0
        marker_sphere.ns = "connected_markers"
        marker_sphere.id = 1
        return marker_sphere

    def create_marker_arrow(self, point):
        marker_arrow = Marker()
        marker_arrow.header.frame_id = "odom"
        marker_arrow.type = Marker.ARROW
        marker_arrow.action = Marker.ADD
        # marker_arrow.pose.position = point
        # marker_arrow.pose.orientation = self.orientation
        marker_arrow.scale.x = 10.0
        marker_arrow.scale.y = 1.2
        marker_arrow.scale.z = 1.2
        marker_arrow.color.a = 1.0
        marker_arrow.color.r = 1.0
        marker_arrow.color.g = 0.0
        marker_arrow.color.b = 0.0
        marker_arrow.ns = "connected_markers"
        marker_arrow.id = 2
        return marker_arrow

    def create_camera_marker_arrow(self, point):
        camera_marker_arrow = Marker()
        camera_marker_arrow.header.frame_id = "odom"
        camera_marker_arrow.type = Marker.ARROW
        camera_marker_arrow.action = Marker.ADD
        # marker_arrow.pose.position = point
        # marker_arrow.pose.orientation = self.orientation
        camera_marker_arrow.scale.x = 8.5
        camera_marker_arrow.scale.y = 0.75
        camera_marker_arrow.scale.z = 0.75
        camera_marker_arrow.color.a = 1.0
        camera_marker_arrow.color.r = 0.0
        camera_marker_arrow.color.g = 1.0
        camera_marker_arrow.color.b = 0.0
        camera_marker_arrow.ns = "connected_markers"
        camera_marker_arrow.id = 3
        return camera_marker_arrow


    def publish_markers(self):
        self.counter += 1

        # Throttle marker publishing to every 2nd call (5Hz effective rate)
        if self.counter % 2 != 0:
            return

        marker_array = MarkerArray()
        marker_array.markers = []


        point1 = Point(x=self.position[0], y=self.position[1], z=8.75)
        point2 = Point(x=self.position[0], y=self.position[1], z=5.0)
        point3 = Point(x=self.position[0], y=self.position[1], z=15.0)

        self.marker_line.pose.position = point1
        self.marker_sphere.pose.position = point3
        self.marker_arrow.pose.position = point3
        self.marker_arrow.pose.orientation = self.orientation

        try:
            transform = self.tf_buffer.lookup_transform('map', 'right_camera_center_link', rclpy.time.Time())
            self.camera_marker_arrow.pose.position = point2
            self.camera_marker_arrow.pose.orientation = transform.transform.rotation
        except Exception as e:
            self.camera_marker_arrow.pose.position = point2
            self.camera_marker_arrow.pose.orientation = self.orientation

        marker_array.markers.append(self.marker_line)
        marker_array.markers.append(self.marker_sphere)
        marker_array.markers.append(self.marker_arrow)
        marker_array.markers.append(self.camera_marker_arrow)

        self.marinero_publisher.publish(marker_array)

def main(args=None):

    rclpy.init(args=args)
    marinero_tracker_node = MarineroMarker()
    rclpy.spin(marinero_tracker_node)
    marinero_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()