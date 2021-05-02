#!/bin/python3
import os
from os.path import expanduser
from typing import Sequence
from urllib.request import urlretrieve

import cv2
from motpy import Detection, MultiObjectTracker, NpImage, Box, Track
from motpy.core import setup_logger
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from motpy_ros.srv import motpy_bbox

logger = setup_logger(__name__, 'DEBUG', is_main=True)
home = expanduser("~")
download_path = home+'/.cache/motpy_ros/face_tracking/'

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

class motpy2darknet:
    def __init__(self):

        ## By run() function --------------------------------
        self.model_spec = {'order_pos': 1, 'dim_pos': 2,
                            'order_size': 0, 'dim_size': 2,
                            'q_var_pos': 5000., 'r_var_pos': 0.1}

        rospy.init_node('motpy_ros')

        self.width = rospy.get_param("~frame_size/width", 360)
        self.height = rospy.get_param("~frame_size/height", 240)
        print( self.width, self.height)

        self.dt = 1 / 60.0  # assume 15 fps
        self.tracker = MultiObjectTracker(dt=self.dt, model_spec=self.model_spec)

        self.motpy_detector = FaceDetector()
        self.bridge = CvBridge()

        rospy.Subscriber("image_raw",Image,self.process_image_ros1)
        self.pub_image = rospy.Publisher("motpy/image_raw",Image,queue_size=1)
        rospy.spin()

    def darknet2tracks(self, bboxes):
        out_detections = []
        for bbox in bboxes.bounding_boxes:
            out_detections.append(Track(id=str(bbox.id),box=[bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax], score=bbox.probability))
        return out_detections

    def face_detect2darknet(self, track):
        one_box = BoundingBox()

        one_box.id = 0
        one_box.Class = "face"
        one_box.xmin = int(track.box[0])
        one_box.ymin = int(track.box[1])
        one_box.xmax = int(track.box[2])
        one_box.ymax = int(track.box[3])
        one_box.probability = float(track.score)
        return one_box

    def process_image_ros1(self, msg):
        try:
            detect2darknet = BoundingBoxes()

            frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            frame = cv2.resize(frame, dsize=(self.width,self.height))
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))

            detections = self.motpy_detector.process_image(frame)
            print("0")
            for detection in detections:
                detect2darknet.bounding_boxes.append(self.face_detect2darknet(detection))

            print(detections)
            for det in detections:
                draw_detection(frame, det)            
            # tracking ------------------------------------------------------------
            try:
                track_faces_module = rospy.ServiceProxy('detect2tracking', motpy_bbox)
                track_bboxes = track_faces_module(detect2darknet).tracking_bboxes

                tracks = self.darknet2tracks(track_bboxes)
                for track in tracks:
                    draw_track(frame,track)
            except:
                pass
            # preview the boxes on frame----------------------------------------
            
            cv2.imshow('frame', frame)
            cv2.waitKey(int(1000 * self.dt))

        except Exception as err:
            print(err)

def ros_main(args = None):
    os.makedirs(download_path, exist_ok=True)
    motpy2darknet_class = motpy2darknet()

if __name__ == "__main__":
    ros_main()