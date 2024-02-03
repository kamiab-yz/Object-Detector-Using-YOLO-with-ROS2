#!/usr/bin/env python3

"""
@Author: Kamyab Yazdipaz
"""

import rclpy                        # ROS2 Python interface library
from rclpy.node import Node         # ROS2 Node class
from sensor_msgs.msg import Image   # Image message type
from cv_bridge import CvBridge      # Class for converting images between ROS and OpenCV
import cv2                          # OpenCV library for image processing

"""
Create a publisher node
"""
class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # Initialize the parent class of the ROS2 node
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)  # Create a publisher object (message type, topic name, queue length)
        self.timer = self.create_timer(0.05, self.timer_callback)         # Create a timer (period in seconds, callback function to be executed periodically)
        self.cap = cv2.VideoCapture(0)                                   # Create a video capture object to drive the camera to capture images (camera device number)
        self.cv_bridge = CvBridge()                                      # Create an image conversion object for later converting OpenCV images to ROS image messages

    def timer_callback(self):
        ret, frame = self.cap.read()                                     # Read the image frame by frame
        
        if ret == True:                                                  # If the image is successfully read
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))             # Publish the image message

        self.get_logger().info('Publishing video frame')                 # Log output, indicating that the image topic has been published

def main(args=None):                                 # Main function for ROS2 node entry point
    rclpy.init(args=args)                            # Initialize the ROS2 Python interface
    node = ImagePublisher("topic_webcam_pub")        # Create and initialize a ROS2 node object
    rclpy.spin(node)                                 # Loop and wait for ROS2 to exit
    node.destroy_node()                              # Destroy the node object
    rclpy.shutdown()                                 # Close the ROS2 Python interface
