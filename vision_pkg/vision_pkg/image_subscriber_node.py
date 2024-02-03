#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Author: Kamyab Yazdipaz
"""

import rclpy                            # ROS2 Python interface library
from rclpy.node import Node             # ROS2 Node class
from sensor_msgs.msg import Image       # Image message type
from cv_bridge import CvBridge          # Class for converting images between ROS and OpenCV
import cv2                              # OpenCV library for image processing
import numpy as np                      # Python library for numerical computations
import random
from ultralytics import YOLO

# opening the file in read mode
my_file = open("/home/kamiab/vision_ws/src/vision_pkg/vision_pkg/utils/coco.txt", "r")

# reading the file
data = my_file.read()
# replacing end splitting the text | when newline ('\n') is seen.
class_list = data.split("\n")
my_file.close()

# print(class_list)

# Generate random colors for class list
detection_colors = []
for i in range(len(class_list)):
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    detection_colors.append((b, g, r))

# load a pretrained YOLOv8n model
model = YOLO("weights/yolov8n.pt", "v8")

# Vals to resize video frames | small frame optimise the run
frame_wid = 640
frame_hyt = 480


"""
Create a subscriber node
"""
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # Initialize the parent class of the ROS2 node
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     # Create a subscriber object (message type, topic name, subscriber callback function, queue length)
        self.cv_bridge = CvBridge()                             # Create an image conversion object for converting between OpenCV images and ROS image messages

    def object_detect(self, image):
        #  resize the frame | small frame optimise the run
        # frame = cv2.resize(frame, (frame_wid, frame_hyt))

        # Predict on image
        detect_params = model.predict(source=[image], conf=0.45, save=False, verbose=False)


        # Convert tensor array to numpy
        DP = detect_params[0].cpu().numpy()
        #print(DP)

        if len(DP) != 0:
            for i in range(len(detect_params[0])):
                #print(i)

                boxes = detect_params[0].boxes
                box = boxes[i]  # returns one box
                clsID = box.cls.cpu().numpy()[0]
                conf = box.conf.cpu().numpy()[0]
                bb = box.xyxy.cpu().numpy()[0]

                cv2.rectangle(
                    image,
                    (int(bb[0]), int(bb[1])),
                    (int(bb[2]), int(bb[3])),
                    detection_colors[int(clsID)],
                    3,
                )

                # Display class name and confidence
                font = cv2.FONT_HERSHEY_COMPLEX
                cv2.putText(
                    image,
                    class_list[int(clsID)] + " " + str(round(conf, 3)) + "%",
                    (int(bb[0]), int(bb[1]) - 10),
                    font,
                    1,
                    (255, 255, 255),
                    2,
                )


        # Display the resulting frame

        
        cv2.imshow("object", image)                             # Display the processed image with OpenCV
        cv2.waitKey(10)



    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # Log output indicating entry into the callback function
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # Convert the ROS image message to an OpenCV image
        self.object_detect(image)                               # Detect apple


def main(args=None):                                        # Main function for ROS2 node entry point
    rclpy.init(args=args)                                   # Initialize the ROS2 Python interface
    node = ImageSubscriber("topic_webcam_sub")              # Create and initialize a ROS2 node object
    rclpy.spin(node)                                        # Loop and wait for ROS2 to exit
    node.destroy_node()                                     # Destroy the node object
    rclpy.shutdown()                                        # Close the ROS2 Python interface
