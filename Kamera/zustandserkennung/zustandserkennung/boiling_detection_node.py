#!usr/bin/env

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import time
import cv2
from ultralytics import YOLO
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


def get_sector(x,y,img_shape,grid):
    num_row,num_col = [int(x) for x in grid.split("x")]
    row_sec_size = img_shape[0] //num_row
    col_sec_size = img_shape[1] //num_col
    row = y //row_sec_size
    col = x//col_sec_size
    return row,col

class Boiling_detection_node(Node):
    def __init__(self):
        super().__init__("Boiling_detection_node")

        # Define the range: from 0.0 to 1.0 (inclusive)
        float_range = FloatingPointRange(
            from_value=0.0,
            to_value=1.0,
            step=0.0  # Step of 0.0 means "any value within range"
        )

        self.declare_parameter('confidance',value=0.4,descriptor=ParameterDescriptor(description="model confidance value from 0 to 1",floating_point_range=[float_range]))
        self.declare_parameter("yolo_model",os.path.join(get_package_share_directory("zustandserkennung"),"models","boil_detection.pt"))
        self.declare_parameter('show_img',True)
        self.declare_parameter("grid", value="2x2")
        self.grid_format = self.get_parameter("grid").get_parameter_value().string_value
        self.nrow, self.ncol = [int(x) for x in self.grid_format.split("x")]

        self.confidance = float(self.get_parameter("confidance").value)#get_parameter_value().string_value)
        self.model_filepath = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.show_imgs = self.get_parameter('show_img').get_parameter_value().bool_value
        self.model = YOLO(self.model_filepath)
        self.bridge = CvBridge()
        self.cam_sub = self.create_subscription(Image,"/camera/color/image_raw",self.image_callback,10)

        self.Zustaende = {}
        self.unstable_zustaende = {}
        self.DETECTION_THRESHOLD = 3

        for row in range(self.nrow):
            for col in range(self.ncol):
                key = f"Platte({row}|{col})"
                self.Zustaende[key] = ["leer", time.monotonic()]
                self.unstable_zustaende[key] = {"state": "leer", "count": 0}

        if self.show_imgs:
            self.img_pub = self.create_publisher(Image,"/image_detected_object",10)
        self.get_logger().info("Detection_Pub Node started sucsessfully")

    def image_callback(self,img_msg=Image):
        if img_msg != None:

            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            results = self.model(cv_image, conf = self.confidance)
            annotated_frame = results[0].plot()

            u, v, _ = annotated_frame.shape
            line_thickness = int(u / 110)

            for row in range(1, self.nrow):
                cv2.line(annotated_frame, (0, int(row * u / self.nrow)), (v, int(row * u / self.nrow)), color=(0, 0, 0), thickness=line_thickness)
            for col in range(1, self.ncol):
                cv2.line(annotated_frame, (int(col * v / self.ncol), 0), (int(col * v / self.ncol), u), color=(0, 0, 0), thickness=line_thickness)

            herdplatte_idx = 0
            for row in range(self.nrow):
                for col in range(self.ncol):
                    herdplatte_idx += 1
                    key = f"Platte({row}|{col})"
                    zustand, zeit = self.Zustaende[key]
                    elapsed = time.monotonic() - zeit

                    # Dynamische Maße je nach Bildgröße und Grid
                    sector_height = u / self.nrow
                    sector_width = v / self.ncol
                    font_scale = min(sector_width, sector_height) / 400  # passt Größe an Grid & Bild an
                    thickness = max(1, int(min(u, v) / 600))  # sicherstellen, dass immer sichtbar
                    y_offset = int(sector_height * 0.1)  # ca. oberes Drittel im Sektor

                    x_pos = int(col * sector_width + 10)  # kleine linke Einrückung
                    y_pos = int(row * sector_height + y_offset)

                    text = f"Platte{herdplatte_idx}: {zustand} seit {elapsed:.1f}s"
                    cv2.putText(annotated_frame, text, (x_pos, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)

            if results[0].obb is not None:
                boxes = results[0].obb
                xywhr = boxes.xywhr.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                classes = boxes.cls.cpu().numpy().astype(int)
                class_names = results[0].names

                # temporäres Mapping Platte -> erkannter Zustand in diesem Frame
                erkannte_zustaende = {}

                for i in range(len(xywhr)):
                    x_center = int(xywhr[i][0])
                    y_center = int(xywhr[i][1])
                    class_id = classes[i]
                    class_name = class_names[class_id]
                    row, col = get_sector(x_center, y_center, (u, v), self.grid_format)
                    platte_key = f"Platte({row}|{col})"
                    erkannte_zustaende[platte_key] = class_name

                    #cv2.drawMarker(annotated_frame, (x_center, y_center), color=(0, 0, 255), markerSize=100, thickness=20)

                for key in self.Zustaende:
                    erkannter_zustand = erkannte_zustaende.get(key, "leer")
                    aktueller_zustand = self.Zustaende[key][0]

                    if erkannter_zustand == aktueller_zustand:
                        self.unstable_zustaende[key] = {"state": erkannter_zustand, "count": 0}
                    else:
                        if self.unstable_zustaende[key]["state"] == erkannter_zustand:
                            self.unstable_zustaende[key]["count"] += 1
                        else:
                            self.unstable_zustaende[key] = {"state": erkannter_zustand, "count": 1}

                        if self.unstable_zustaende[key]["count"] >= self.DETECTION_THRESHOLD:
                            print(f"{key} Zustand geändert von {aktueller_zustand} zu {erkannter_zustand}")
                            self.Zustaende[key] = [erkannter_zustand, time.monotonic()]
                            self.unstable_zustaende[key]["count"] = 0

            self.publish_image(annotated_frame)

# old
            # cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            # results = self.model(cv_image, conf = self.confidance)
            # annotated_frame = results[0].plot()

            # if results[0].obb and len(results[0].obb.data) > 0:
            #     for i in range(len(results[0].obb.data)):
            #         center_x,center_y = results[0].obb.data[i,:2]
            #         self.get_logger().info(f"Object detected: center_x: {center_x} | center_y = {center_y}")
            #         cv2.drawMarker(annotated_frame,(int(center_x),int(center_y)),color=(0,0,255))
            #     if self.show_imgs:
            #         self.publish_image(annotated_frame)

    def publish_image(self,out_img):
        ros_image = self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8')
        if self.show_imgs:
            self.img_pub.publish(ros_image)

    



def main(args=None):
    rclpy.init(args=args)
    OBB_Node = Boiling_detection_node()
    rclpy.spin(OBB_Node)
    rclpy.shutdown()
