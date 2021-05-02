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

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

logger = setup_logger(__name__, 'DEBUG', is_main=True)
home = expanduser("~")
download_path = home+'/.cache/face_detector/face_tracking/'

WEIGHTS_URL = 'https://github.com/opencv/opencv_3rdparty/raw/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel'
WEIGHTS_PATH = download_path+'opencv_face_detector.caffemodel'
CONFIG_URL = 'https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt'
CONFIG_PATH = download_path+'deploy.prototxt'

class FaceDetector:
    def __init__(self,
                 weights_url: str = WEIGHTS_URL,
                 weights_path: str = WEIGHTS_PATH,
                 config_url: str = CONFIG_URL,
                 config_path: str = CONFIG_PATH,
                 conf_threshold: float = 0.5) -> None:
        super(FaceDetector, self).__init__()

        self.weights_url = rospy.get_param("~weights_url", WEIGHTS_URL)
        self.config_url = rospy.get_param("~config_url", CONFIG_URL)

        self.weights_path = rospy.get_param("~weights_path", WEIGHTS_PATH)
        self.config_path = rospy.get_param("~config_path", CONFIG_PATH)

        print(weights_url, "\n", config_url)
        print(weights_path, "\n", config_path)

        if not os.path.isfile(weights_path) or not os.path.isfile(config_path):
            logger.debug('downloading model...')
            urlretrieve(self.weights_url, weights_path)
            urlretrieve(self.config_url, config_path)

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

class cv2dnn2darknet:
    def __init__(self):


        rospy.init_node('face_detector')

        self.width = rospy.get_param("~frame_size/width", 360)
        self.height = rospy.get_param("~frame_size/height", 240)
        self.imshow_isview = rospy.get_param("~imshow_isshow", 1)
        print( self.width, self.height)

        self.motpy_detector = FaceDetector()
        self.bridge = CvBridge()

        rospy.Subscriber("camera/color/image_raw",Image,self.process_image_ros1)
        self.pub_image = rospy.Publisher("motpy/image_raw",Image,queue_size=1)
        self.pub = rospy.Publisher('bounding_boxes', BoundingBoxes, queue_size=1)
        rospy.spin()

    def create_d_msgs_box(self, track):
        one_box = BoundingBox()

        one_box.id = 0
        one_box.Class = "face"
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

    def process_image_ros1(self, msg):
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

            if self.imshow_isview:
                cv2.imshow('frame', frame)
                cv2.waitKey(1)

        except Exception as err:
            print(err)

def ros_main(args = None):
    os.makedirs(download_path, exist_ok=True)
    cv2dnn2darknet_class = cv2dnn2darknet()

if __name__ == "__main__":
    ros_main()