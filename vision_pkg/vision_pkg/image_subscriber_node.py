#!/usr/bin/env python3


"""
@Author: Kamyab Yazdipaz
"""

import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import Image       
from cv_bridge import CvBridge          
import cv2                              
import numpy as np                      
import random
from ultralytics import YOLO

# opening the file in read mode
my_file = open("/home/kamiab/vision_ws/src/vision_pkg/vision_pkg/utils/coco.txt", "r")


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
        super().__init__(name)                                 
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     
        self.cv_bridge = CvBridge()                             

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

        
        cv2.imshow("object", image)                             
        cv2.waitKey(10)



    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      
        self.object_detect(image)                              


def main(args=None):                                      
    rclpy.init(args=args)                                  
    node = ImageSubscriber("topic_webcam_sub")             
    rclpy.spin(node)                                        
    node.destroy_node()                                    
    rclpy.shutdown()                                        
