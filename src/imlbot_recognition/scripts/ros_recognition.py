#!/usr/bin/env python
import os, sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

from pathlib import Path
import torch

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='../models/water-bottles.pt')

        self.subscription = self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("SOURCE", img)
        cv2.waitKey(5)

        # Inference
        results = self.model(img)

        r = results.pandas().xyxy[0]

        for i in range(len(r)):
            #print(f'{r.iloc[i]["name"]} : {r.iloc[i]["xmin"]} {int(r.iloc[i]["xmin"])}')
            cv2.rectangle(img, (int(r.iloc[i]["xmax"]), int(r.iloc[i]["ymax"])), (int(r.iloc[i]["xmin"]), int(r.iloc[i]["ymin"])), (0, 0, 255), 2)
            cv2.putText(img, f'{r.iloc[i]["name"]}: {r.iloc[i]["confidence"]:.2f}', (int(r.iloc[i]["xmax"]), int(r.iloc[i]["ymax"])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("RECOGNITION", img)
        cv2.waitKey(4)    

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

