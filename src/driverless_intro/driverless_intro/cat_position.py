#!/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random
import sys

class CatPositionPublisher(Node):
    def __init__(self, track_file):
        super().__init__('cat_position_publisher')
        self.publisher_ = self.create_publisher(Point, '/cat_position', 10)
        self.timer = self.create_timer(1.0, self.publish_position)
        self.cat_position = self.generate_random_position(track_file)
        self.get_logger().info(f'Generated cat position: {self.cat_position}')
    
    def generate_random_position(self, track_file):
        point = Point()
        if track_file == 'acceleration.csv':
            point.x = random.uniform(0, 180)
            point.y = random.uniform(-10, 10)
        elif track_file == 'FSG.csv':
            point.x = random.uniform(-30, 50)
            point.y = random.uniform(-80, 10)
        point.z = 0.0  # Assuming 2D plane
        return point

    def publish_position(self):
        self.publisher_.publish(self.cat_position)
        # self.get_logger().info(f'Publishing cat position: {self.cat_position}')


def main(args=None):
    rclpy.init(args=args)
    track_file = sys.argv[1] if len(sys.argv) > 1 else 'FSG.csv'
    print(f'Track file: {track_file}')
    node = CatPositionPublisher(track_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()