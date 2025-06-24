#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2
import time

def open_camera(start_index=0):
    index = start_index
    while index <= 5:
        try:
            # Try to create a video capture object using the current index
            videoCaptureObject = cv2.VideoCapture(index)
            if not videoCaptureObject.isOpened():
                raise IOError("Cannot open camera index {}".format(index))
            print(f"Camera index {index} opened successfully")
            return videoCaptureObject
        except (IOError, ValueError) as e:
            print(e)
            print(f"Trying next camera index {index+1}")
            index += 1


class CameraSensorPublisher(Node):

    def __init__(self):
        super().__init__("camera_sensor_publisher")
        self.video_publisher = self.create_publisher(Image,"camera/color/image_raw",60)
        self.get_logger().info("camera sensor publisher has started")



def main(args=None):
    rclpy.init(args=args)
    node = CameraSensorPublisher()
    videoCaptureObject = open_camera(0)

    bridgeObject = CvBridge()
    while rclpy.ok():
        return_value, capturedFrame = videoCaptureObject.read()
        if return_value:
            
            imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame,"bgr8")
            node.video_publisher.publish(imageToTransmit)
            time.sleep(0.2)


    rclpy.spin(node)
    rclpy.shutdown()
