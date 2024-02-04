#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Author: Kamyab Yazdipaz
"""

import rclpy                            
from rclpy.node import Node            
from sensor_msgs.msg import Image       # Image message type
from cv_bridge import CvBridge          # Class for converting images between ROS and OpenCV
import cv2                              
import numpy as np                      
import random
from ultralytics import YOLO
from my_custom_interfaces.msg import ObjPlaceLoc #Custom message Interface


# opening the file in read mode
my_file = open("/home/kamiab/vision_ws/src/vision_pkg/vision_pkg/utils/coco.txt", "r") #TODO: change it to your file path


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



class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  
        
        self.declare_parameter("object_name", "bottle")         # Declare the parameter for object and it's place
        self.declare_parameter("place_name", "chair")

        self.obj_name_ = self.get_parameter("object_name").value        # Get the parameters
        self.place_name_ =self.get_parameter("place_name").value

        self.obj_place_info_publisher_ = self.create_publisher(ObjPlaceLoc, "obj_place_info", 10)
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     
        self.cv_bridge = CvBridge()                             
    
    
    def publish_obj_place_info(self, object_name, obj_c_x, obj_c_y, place_name, place_c_x, place_c_y):
        msg = ObjPlaceLoc()
        msg.object_name = object_name
        msg.object_x = obj_c_x
        msg.object_y = obj_c_y
        msg.place_name = place_name
        msg.place_x = place_c_x
        msg.place_y = place_c_y
        self.obj_place_info_publisher_.publish(msg)

    def detect_obj_place(self, image):
        #  resize the frame | small frame optimise the run
        # frame = cv2.resize(frame, (frame_wid, frame_hyt))

        object_name = "None"
        obj_c_x= 0.0
        obj_c_y = 0.0

        place_name = "None"
        place_c_x = 0.0
        place_c_y = 0.0

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
                if class_list[int(clsID)] == self.obj_name_  or class_list[int(clsID)] == self.place_name_:

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
                    
                    centerX = int((bb[0] + bb[2]) / 2)
                    centerY = int((bb[1] + bb[3]) / 2)
                    cv2.circle(image, (centerX, centerY), radius=5, color=(0, 255, 0), thickness=-1)


                    if class_list[int(clsID)] == self.obj_name_:
                        object_name = self.obj_name_
                        obj_c_x = (int(bb[0]) + (int(bb[2])))/2
                        obj_c_y = (int(bb[1]) + (int(bb[3])))/2

                    if class_list[int(clsID)] == self.place_name_:
                        place_name = self.place_name_
                        place_c_x = (int(bb[0]) + (int(bb[2])))/2
                        place_c_y = (int(bb[1]) + (int(bb[3])))/2

            self.publish_obj_place_info(object_name, obj_c_x, obj_c_y, place_name, place_c_x, place_c_y)    


        # Display the resulting frame

        
        cv2.imshow("object", image)                             # Display the processed image with OpenCV
        cv2.waitKey(10)



    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # Log output indicating entry into the callback function
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # Convert the ROS image message to an OpenCV image
        self.detect_obj_place(image)                               


def main(args=None):                                        
    rclpy.init(args=args)                                   
    node = ImageSubscriber("topic_webcam_sub")              
    rclpy.spin(node)                                        
    node.destroy_node()                                    
    rclpy.shutdown()                                        
