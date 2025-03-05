#!/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from driverless_intro.msg import Cone, Track

class ConePublisher(Node):
    def __init__(self):
        super().__init__('cone_publisher')

        self.publisher = self.create_publisher(Marker, '/cone_marker', 10)

        self.subscription = self.create_subscription(
            Track,
            '/cones', 
            self.track_callback,
            10
        )
        
        self.target_subscription = self.create_subscription(
            Cone,
            '/target_cone',
            self.target_cone_callback,
            10
        )
        
        self.cones = []
        self.target_cone = None
        
        # Periodically publish markers
        self.timer = self.create_timer(1.0, self.publish_cones)

    def track_callback(self, msg):
        # Clear the previous cones
        self.cones = []

        # Iterate over the cones in the Track message
        for cone in msg.cones:
            x = cone.position.x
            y = cone.position.y
            z = cone.position.z
            cone_type = cone.type
            knocked_over = cone.knocked_over

            self.cones.append((x, y, z, cone_type, knocked_over))

    def target_cone_callback(self, msg):
        self.target_cone = (msg.position.x, msg.position.y, msg.position.z)

    def publish_cones(self):
        for idx, (x, y, z, cone_type, knocked_over) in enumerate(self.cones):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'cones'
            marker.id = idx  # Unique ID for each cone
            marker.type = Marker.SPHERE  
            marker.action = Marker.ADD
            marker.scale.x = 0.2 
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.color.a = 1.0  # Fully opaque

            # Check if this cone is the target cone
            if self.target_cone and (x, y, z) == self.target_cone:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            else:
                # Set color based on cone type
                if cone_type == 0:  # BLUE
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif cone_type == 1:  # YELLOW
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif cone_type == 2:  # ORANGE_BIG
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                elif cone_type == 3:  # ORANGE_SMALL
                    marker.color.r = 1.0
                    marker.color.g = 0.7
                    marker.color.b = 0.0
                else:  # UNKNOWN
                    marker.color.r = 0.5
                    marker.color.g = 0.5
                    marker.color.b = 0.5

            # Publish the marker for this cone
            self.publisher.publish(marker)

        self.get_logger().info(f"Published {len(self.cones)} cone positions.")

def main(args=None):
    rclpy.init(args=args)
    node = ConePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
