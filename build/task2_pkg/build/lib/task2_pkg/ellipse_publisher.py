import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from .shapeDetection import Shape
import cv2
import numpy as np


class Ellipse(Shape):
    def __init__(self, frame):
        super().__init__()
        self.frame = frame

    def detect_shapes(self):
        frame, contours = super().detect_shapes(self.frame)
        ellipse_contours = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True) 
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter != 0:
                circularity = 4 * np.pi * area / (perimeter**2)
                if circularity < 0.9 and len(approx) > 10 :
                    ellipse_contours.append(contour)
        return ellipse_contours
                
    
class EllipsePublisherNode(Node): 
    def __init__(self):
        super().__init__("ellipse_node") 
        self.cap = cv2.VideoCapture(0)

        self.publisher_ = self.create_publisher(String, "ellipse_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_shapes)
        self.get_logger().info("Publisher has been started.")

    def publish_shapes(self):
        ret, frame = self.cap.read()

        ellipses = Ellipse(frame)
        ellipses_num = ellipses.find_shape_count(ellipses.detect_shapes())

        frame_with_shapes = ellipses.contour_shapes(frame, ellipses.detect_shapes())

        cv2.imshow("Shapes Detected", frame_with_shapes)
        cv2.waitKey(1) 
        
        msg = String()
        msg.data = "Ellipse: " + str(ellipses_num)
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = EllipsePublisherNode() 
    rclpy.spin(node)
    node.cap.release() 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
