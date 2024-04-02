import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from shapeDetection import Shape
import cv2
import numpy as np


class Circle(Shape):
    def __init__(self, frame):
        super().__init__()
        self.frame = frame

    def detect_shapes(self):
        frame, contours = super().detect_shapes(self.frame)
        circle_contours = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True) 
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter != 0:
                circularity = 4 * np.pi * area / (perimeter**2)
                if circularity > 0.9 and len(approx) > 10 :
                    circle_contours.append(contour)
        return circle_contours
                
    
class CirclePublisherNode(Node): 
    def __init__(self):
        super().__init__("circle_node") 
        self.cap = cv2.VideoCapture(0)

        self.publisher_ = self.create_publisher(String, "circle_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_shapes)
        self.get_logger().info("Publisher has been started.")

    def publish_shapes(self):
        ret, frame = self.cap.read()

        circles = Circle(frame)
        circles_num = circles.find_shape_count(circles.detect_shapes())

        frame_with_shapes = circles.contour_shapes(frame, circles.detect_shapes())

        cv2.imshow("Shapes Detected", frame_with_shapes)
        cv2.waitKey(1) 
        
        msg = String()
        msg.data = "Circle: " + str(circles_num)
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisherNode() 
    rclpy.spin(node)
    node.cap.release() 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
