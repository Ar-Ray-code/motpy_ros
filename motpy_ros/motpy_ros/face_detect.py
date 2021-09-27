#!/bin/python3
import os
from os.path import expanduser
from typing import Sequence
from urllib.request import urlretrieve

import cv2
from motpy import Detection,  NpImage, Box, Track
from motpy.core import setup_logger
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from bboxes_ex_msgs.msg import BoundingBoxes
from bboxes_ex_msgs.msg import BoundingBox

logger = setup_logger(__name__, 'DEBUG', is_main=True)
home = expanduser("~")
download_path = home+'/.cache/face_detector/face_tracking/'

WEIGHTS_URL = 'https://github.com/opencv/opencv_3rdparty/raw/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel'
WEIGHTS_PATH = download_path+'opencv_face_detector.caffemodel'
CONFIG_URL = 'https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt'
CONFIG_PATH = download_path+'deploy.prototxt'

class FaceDetector(BaseObjectDetector):
    def __init__(self,
                 weights_url: str = WEIGHTS_URL,
                 weights_path: str = WEIGHTS_PATH,
                 config_url: str = CONFIG_URL,
                 config_path: str = CONFIG_PATH,
                 conf_threshold: float = 0.3) -> None:
        super(FaceDetector, self).__init__()


        print(weights_url, "\n", config_url)
        print(weights_path, "\n", config_path)

        if not os.path.isfile(weights_path) or not os.path.isfile(config_path):
            logger.debug('downloading model...')
            urlretrieve(weights_url, weights_path)
            urlretrieve(config_url, config_path)

        self.net = cv2.dnn.readNetFromCaffe(config_path, weights_path)

        # specify detector hparams
        self.conf_threshold = conf_threshold

    def process_image(self, image: NpImage) -> Sequence[Detection]:
        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], False, False)
        self.net.setInput(blob)
        detections = self.net.forward()

        # convert output from OpenCV detector to tracker expected format [xmin, ymin, xmax, ymax]
        out_detections = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                xmin = int(detections[0, 0, i, 3] * image.shape[1])
                ymin = int(detections[0, 0, i, 4] * image.shape[0])
                xmax = int(detections[0, 0, i, 5] * image.shape[1])
                ymax = int(detections[0, 0, i, 6] * image.shape[0])
                out_detections.append(Detection(box=[xmin, ymin, xmax, ymax], score=confidence))
            
        return out_detections

class cv2dnn2darknet(Node):
    def __init__(self):

        ## RCLPY 
        super().__init__('face_detect')

        # ==============================================================

        self.declare_parameter('frame_size/width',360)
        self.declare_parameter('frame_size/height',240)
        self.declare_parameter('imshow_isshow',1)

        self.declare_parameter('weights_url', WEIGHTS_URL)
        self.declare_parameter('config_url', CONFIG_URL)
        self.declare_parameter('weights_path', WEIGHTS_PATH)
        self.declare_parameter('config_path', CONFIG_PATH)

        # ==============================================================

        self.width = self.get_parameter('frame_size/width').value
        self.height = self.get_parameter('frame_size/height').value
        self.imshow_isshow = self.get_parameter('imshow_isshow').value

        self.weights_url = self.get_parameter('weights_url').value
        self.config_url = self.get_parameter('config_url').value

        self.weights_path = self.get_parameter('weights_path').value
        self.config_path = self.get_parameter('config_path').value

        # ==============================================================

        self.sub = self.create_subscription(Image,"camera/color/image_raw",self.process_image_ros2, 10)
        self.pub_image = self.create_publisher(Image,"motpy/image_raw", 10)
        self.pub = self.create_publisher(BoundingBoxes,'bounding_boxes', 10)

        # ==============================================================
        print(self.weights_url, "\n", self.config_url)

        self.motpy_detector = FaceDetector(weights_url=self.weights_url, weights_path=self.weights_path, config_url=self.config_url, config_path=self.config_path)
        self.bridge = CvBridge()

    def create_d_msgs_box(self, track):
        one_box = BoundingBox()

        one_box.id = 0
        one_box.class_id = "face"
        one_box.probability = float(track.score)
        one_box.xmin = int(track.box[0])
        one_box.ymin = int(track.box[1])
        one_box.xmax = int(track.box[2])
        one_box.ymax = int(track.box[3])

        return one_box

    def publish_d_msgs(self, tracks, img_msg):
        
        boxes = BoundingBoxes()
        boxes.header = img_msg.header
        
        for track in tracks:
            boxes.bounding_boxes.append(self.create_d_msgs_box(track))

        print("boxes--------------------")
        for box_print in boxes.bounding_boxes:
            print(box_print)
        print("\n\n")
        
        self.pub.publish(boxes)

    def process_image_ros2(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            frame = cv2.resize(frame, dsize=(self.width,self.height))
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))

            # # run face detector on current frame
            
            detections = self.motpy_detector.process_image(frame)
            self.publish_d_msgs(detections, msg)

            # preview the boxes on frame----------------------------------------
            print(detections)
            for det in detections:
                draw_detection(frame, det)

            if self.imshow_isshow:
                cv2.imshow('frame', frame)
                cv2.waitKey(1)

        except Exception as err:
            print(err)

def ros_main(args = None):
    os.makedirs(download_path, exist_ok=True)
    rclpy.init(args=args)

    cv2dnn2darknet_class = cv2dnn2darknet()
    rclpy.spin(cv2dnn2darknet_class)

    cv2dnn2darknet_class.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros_main()