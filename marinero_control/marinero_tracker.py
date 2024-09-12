#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry

class MarineroMarker(Node):
    def __init__(self):
        super().__init__("marinero_marker")
        self.odom_sub = self.create_subscription(Odometry, '/marinero/odom', self.location_callback, 10)
        self.marinero_publisher = self.create_publisher(MarkerArray, '/marinero_tracker', 10)
        self.marker_timer = self.create_timer(0.1, self.publish_markers)
        self.position = (0.0, 0.0)
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Reuse markers by creating them once
        self.marker_line = self.create_marker_line(Point(), Point())
        self.marker_sphere = self.create_marker_sphere(Point())
        self.marker_arrow = self.create_marker_arrow(Point())
        
        # Counter for throttling publishing
        self.counter = 0
        
    def location_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.orientation = msg.pose.pose.orientation
        
    def create_marker_line(self, point1, point2):
        marker_line = Marker()
        marker_line.header.frame_id = 'odom'
        marker_line.type = Marker.LINE_STRIP
        marker_line.action = Marker.ADD
        # marker_line.points = [point1, point2]
        marker_line.scale.x = 0.7
        marker_line.color.a = 1.0
        marker_line.color.r = 1.0
        marker_line.color.g = 0.0
        marker_line.color.b = 0.0
        return marker_line
    
    def create_marker_sphere(self, point):
        marker_sphere = Marker()
        marker_sphere.header.frame_id = 'odom'
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
        return marker_sphere
    
    def create_marker_arrow(self, point):
        marker_arrow = Marker()
        marker_arrow.header.frame_id = 'odom'
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
        return marker_arrow

    def publish_markers(self):
        self.counter += 1
        
        # Throttle marker publishing to every 2nd call (5Hz effective rate)
        if self.counter % 2 != 0:
            return
        
        marker_array = MarkerArray()
        
        point1 = Point(x=self.position[0], y=self.position[1], z=2.0)
        point2 = Point(x=self.position[0], y=self.position[1], z=20.0)
        
        self.marker_line.points = [point1, point2]
        self.marker_sphere.pose.position = point2
        self.marker_arrow.pose.position = point2
        self.marker_arrow.pose.orientation = self.orientation
        
        self.marker_line.ns = "connected_markers"
        self.marker_line.id = 0
        self.marker_sphere.ns = "connected_markers"
        self.marker_sphere.id = 1
        self.marker_arrow.ns = "connected_markers"
        self.marker_arrow.id = 2
        
        # Append markers to array
        marker_array.markers.append(self.marker_line)
        marker_array.markers.append(self.marker_sphere)
        marker_array.markers.append(self.marker_arrow)
        
        # Publish the MarkerArray
        self.marinero_publisher.publish(marker_array) 
        
def main(args=None):
    
    rclpy.init(args=args)
    marinero_tracker_node = MarineroMarker()
    rclpy.spin(marinero_tracker_node)
    marinero_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        