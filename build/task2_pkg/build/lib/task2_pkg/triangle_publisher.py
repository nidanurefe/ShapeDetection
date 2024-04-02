#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from .shapeDetection import Shape
import cv2


class Triangle(Shape):
    def __init__(self, frame):
        super().__init__()
        self.frame = frame

    def detect_shapes(self):
        frame, contours = super().detect_shapes(self.frame)
        triangle_contours = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True) 
            if len(approx) == 3:
                triangle_contours.append(approx)
        return triangle_contours
                
    
class TrianglePublisherNode(Node): 
    def __init__(self):
        super().__init__("triangle_node") 
        self.cap = cv2.VideoCapture(0)

        self.publisher_ = self.create_publisher(String, "triangle_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_shapes)
        self.get_logger().info("Publisher has been started.")

    def publish_shapes(self):
        ret, frame = self.cap.read()

        triangles = Triangle(frame)
        triangle_num = triangles.find_shape_count(triangles.detect_shapes())

        frame_with_shapes = triangles.contour_shapes(frame, triangles.detect_shapes())

        cv2.imshow("Shapes Detected", frame_with_shapes)
        cv2.waitKey(1) 
        
        msg = String()
        msg.data = "Triangle: " + str(triangle_num)
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = TrianglePublisherNode() 
    rclpy.spin(node)
    node.cap.release() 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
