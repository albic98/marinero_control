#usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from ament_index_python.packages import get_package_share_directory

package_name = "marinero_simulations"
package_path = get_package_share_directory(package_name)

class MarkerSpawner(Node):
    def __init__(self):
        super().__init__("marker_spawner")

        self.odom_sub = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 10)

        self.marker_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.entity_state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        self.sdf_file_path = os.path.join(package_path,"models/gazebo_marker/model.sdf")
        with open(self.sdf_file_path, "r") as f:
            self.sdf_content = f.read()

        self.marker_height = 2.25
        self.robot_pose = None
        self.marker_spawned = False
        self.marker_spawn_in_progress = False
        self.marker_name = "gazebo_marker"

        self.get_logger().info("Waiting for services...")
        self.marker_client.wait_for_service(timeout_sec=1.0)
        self.entity_state_client.wait_for_service(timeout_sec=1.0)

    def spawn_marker(self):

        if self.marker_spawn_in_progress or self.marker_spawned:
            return
        self.marker_spawn_in_progress = True 

        marker_request = SpawnEntity.Request()
        marker_request.name = self.marker_name
        marker_request.xml = self.sdf_content
        marker_request.robot_namespace = "marinero"
        marker_request.initial_pose.position.x = self.robot_pose.pose.pose.position.x
        marker_request.initial_pose.position.y = self.robot_pose.pose.pose.position.y
        marker_request.initial_pose.position.z = self.marker_height
        marker_request.initial_pose.orientation = self.robot_pose.pose.pose.orientation

        future = self.marker_client.call_async(marker_request)
        future.add_done_callback(self.spawn_marker_response_callback)

    def spawn_marker_response_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info("Marker spawned successfully!")
                self.marker_spawned = True
            else:
                self.get_logger().error("Failed to spawn marker.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        finally:
            self.marker_spawn_in_progress = False
            
    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg

        if not self.marker_spawned:
            if self.robot_pose is not None:
                self.spawn_marker()
        else:
            self.update_marker_position()

    def update_marker_position(self):
        marker_state = EntityState()
        marker_state.name = self.marker_name
        marker_state.pose.position.x = self.robot_pose.pose.pose.position.x
        marker_state.pose.position.y = self.robot_pose.pose.pose.position.y
        marker_state.pose.position.z = self.marker_height
        marker_state.pose.orientation = self.robot_pose.pose.pose.orientation


        state_request = SetEntityState.Request()
        state_request.state = marker_state

        future = self.entity_state_client.call_async(state_request)

    # def update_marker_position_response_callback(self, future):
    #     try:
    #         result = future.result()
    #         if result is not None:
    #             self.get_logger().info("Marker position updated!")
    #         else:
    #             self.get_logger().error("Failed to update marker position.")
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()