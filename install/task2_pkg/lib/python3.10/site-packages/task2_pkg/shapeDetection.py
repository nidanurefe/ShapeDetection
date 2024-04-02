import cv2

class Shape:
        
    def detect_shapes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        thresh = cv2.threshold(blurred, 60, 225, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        return frame, contours

        
    def find_shape_count(self, contour_list):
        print(len(contour_list))
        return len(contour_list)

    def contour_shapes(self, frame, contours):
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 5) 
        return frame  
   

 