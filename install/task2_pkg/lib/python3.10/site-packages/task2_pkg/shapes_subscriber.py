#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class ShapesSubscriberNode(Node): 
    def __init__(self):
        super().__init__("shapes") 
        self.subscriber_ = self.create_subscription(
            String, "triangle_topic", self.callback_triangle, 10)
        self.subscriber_ = self.create_subscription(
            String, "circle_topic", self.callback_circle, 10)
        self.subscriber_ = self.create_subscription(
            String, "rectangle_topic", self.callback_rectangle, 10)
        self.subscriber_ = self.create_subscription(
            String, "ellipse_topic", self.callback_ellipse, 10)
        self.get_logger().info("Infos are getting received.")

    def callback_triangle(self, msg):
        self.get_logger().info("Triangle: " + msg.data)

    def callback_circle(self, msg):
        self.get_logger().info("Circle: " + msg.data)

    def callback_rectangle(self, msg):
        self.get_logger().info("Rectangle: " + msg.data)

    def callback_ellipse(self, msg):
        self.get_logger().info("Ellipse: " + msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ShapesSubscriberNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

