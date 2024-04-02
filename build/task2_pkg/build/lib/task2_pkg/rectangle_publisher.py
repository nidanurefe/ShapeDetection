import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from .shapeDetection import Shape
import cv2


class Rectangle(Shape):
    def __init__(self, frame):
        super().__init__()
        self.frame = frame

    def detect_shapes(self):
        frame, contours = super().detect_shapes(self.frame)
        rectangle_contours = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True) 
            if len(approx) == 4:
                rectangle_contours.append(approx)
        return rectangle_contours
                
    
class RectanglePublisherNode(Node): 
    def __init__(self):
        super().__init__("rectangle_node") 
        self.cap = cv2.VideoCapture(0)

        self.publisher_ = self.create_publisher(String, "rectangle_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_shapes)
        self.get_logger().info("Publisher has been started.")

    def publish_shapes(self):
        ret, frame = self.cap.read()

        rectangles = Rectangle(frame)
        rectangles_num = rectangles.find_shape_count(rectangles.detect_shapes())

        frame_with_shapes = rectangles.contour_shapes(frame, rectangles.detect_shapes())

        cv2.imshow("Shapes Detected", frame_with_shapes)
        cv2.waitKey(1) 
        
        msg = String()
        msg.data = "Rectangle" + str(rectangles_num)
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = RectanglePublisherNode() 
    rclpy.spin(node)
    node.cap.release() 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
