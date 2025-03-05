#!/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class CatVisualizer(Node):
    def __init__(self):
        super().__init__('cat_visualizer')

        meshes_path = "package://driverless_intro/meshes/cat.dae"

        self.marker_pub = self.create_publisher(Marker, '/cat_marker', 10)

        self.position_sub = self.create_subscription(
            Point, '/cat_position', self.position_callback, 10)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "cat"
        self.marker.id = 0
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD

        self.marker.mesh_resource = meshes_path
        self.marker.mesh_use_embedded_materials = True

        self.marker.scale.x = 0.25 
        self.marker.scale.y = 0.25
        self.marker.scale.z = 0.25

        # Set the orientation to make the cat stand upright
        self.marker.pose.orientation.x = 0.8
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

    def position_callback(self, msg):
        """
        Callback function to update the cat's position based on the /cat_position topic.
        """
        self.marker.pose.position.x = msg.x
        self.marker.pose.position.y = msg.y
        self.marker.pose.position.z = msg.z

        self.marker.header.stamp = self.get_clock().now().to_msg()

        self.marker_pub.publish(self.marker)
        # self.get_logger().info(f"Published cat marker at ({msg.x}, {msg.y}, {msg.z})")

def main(args=None):
    rclpy.init(args=args)
    cat_visualizer = CatVisualizer()
    rclpy.spin(cat_visualizer)
    cat_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()