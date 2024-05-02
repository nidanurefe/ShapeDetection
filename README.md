This repository contains ROS2 packages with publisher and subscriber nodes for different shapes: ellipse, triangle, circle and rectangle.
These nodes demonstrate how to publish shape messages and subscribe to them using ROS 2.

**Prequisites**
ROS2: Install ROS 2 Humble.

**Installation**
1. Clone this repository into your ROS 2 workspace:
   ``` git clone https://github.com/your_username/ros2-shapes.git ```

2. Build the packages:
 ``` colcon build  ```

**Usage**
Launch each shaoe node individually using ROS 2 launch files.  

**Packages**
shapes_msgs: Contains custom message definitions for different shapes. »
ellipse_publisher: Publishes ellipse messages. »
triangle_publisher: Publishes triangle messages.
circle_publisher: Publishes circle messages.
rectangle_publisher: Publishes rectangle messages.
shapes_subscriber: Subscribes to shape publishers.
