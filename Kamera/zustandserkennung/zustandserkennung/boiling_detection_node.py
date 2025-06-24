#!usr/bin/env

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import cv2
from ultralytics import YOLO
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

class Boiling_detection_node(Node):
    def __init__(self):
        super().__init__("Boiling_detection_node")

        # Define the range: from 0.0 to 1.0 (inclusive)
        float_range = FloatingPointRange(
            from_value=0.0,
            to_value=1.0,
            step=0.0  # Step of 0.0 means "any value within range"
        )

        self.declare_parameter('confidance',value=0.8,descriptor=ParameterDescriptor(description="model confidance value from 0 to 1",floating_point_range=[float_range]))
        self.declare_parameter("yolo_model",os.path.join(get_package_share_directory("zustandserkennung"),"models","bolt_detection.pt"))
        self.declare_parameter('show_img',False)

        self.confidance = float(self.get_parameter("confidance").value)#get_parameter_value().string_value)
        self.model_filepath = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.show_imgs = self.get_parameter('show_img').get_parameter_value().bool_value
        self.model = YOLO(self.model_filepath)

        self.bridge = CvBridge()

        self.cam_sub = self.create_subscription(Image,"/camera/color/image_raw",self.image_callback,10)

        if self.show_imgs:
            self.img_pub = self.create_publisher(Image,"/image_detected_object",10)
        self.get_logger().info("Detection_Pub Node started sucsessfully")

    def image_callback(self,img_msg=Image):
        if img_msg != None:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            results = self.model(cv_image, conf = self.confidance)
            annotated_frame = results[0].plot()

            if results[0].obb and len(results[0].obb.data) > 0:
                for i in range(len(results[0].obb.data)):
                    center_x,center_y = results[0].obb.data[i,:2]
                    self.get_logger().info(f"Object detected: center_x: {center_x} | center_y = {center_y}")
                    cv2.drawMarker(annotated_frame,(int(center_x),int(center_y)),color=(0,0,255))
                if self.show_imgs:
                    self.publish_image(annotated_frame)

    def publish_image(self,out_img):
        ros_image = self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8')
        if self.show_imgs:
            self.img_pub.publish(ros_image)

    



def main(args=None):
    rclpy.init(args=args)
    OBB_Node = Boiling_detection_node()
    rclpy.spin(OBB_Node)
    rclpy.shutdown()