import os
from os.path import expanduser
from typing import Sequence
from urllib.request import urlretrieve

import cv2
from motpy import Detection, MultiObjectTracker, NpImage, Box, Track
from motpy.core import setup_logger
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

logger = setup_logger(__name__, 'DEBUG', is_main=True)


class motpy2darknet(Node):
    def __init__(self):

        ## By run() function --------------------------------
        self.model_spec = {'order_pos': 1, 'dim_pos': 2,
                            'order_size': 0, 'dim_size': 2,
                            'q_var_pos': 5000., 'r_var_pos': 0.1}

        self.dt = 1 / 10.0  # assume 15 fps
        self.tracker = MultiObjectTracker(dt=self.dt, model_spec=self.model_spec)

        # self.motpy_detector = FaceDetector()

        ## RCLPY 
        super().__init__('motpy_ros')
        self.pub = self.create_publisher(BoundingBoxes,"bounding_boxes", 1)
        self.sub = self.create_subscription(BoundingBoxes,"darknet_ros/bounding_boxes",self.process_boxes_ros2,1)
        self.sub_img = self.create_subscription(Image,"color/image_raw",self.process_image_ros2,1)

        self.bridge = CvBridge()

    def bboxes2out_detections(self,bboxes):
        # for(bbox)
        out_detections = []
        out_class_id = []

        for bbox in bboxes.bounding_boxes:
            xmin = bbox.xmin
            xmax = bbox.xmax
            ymin = bbox.ymin
            ymax = bbox.ymax
            confidence = bbox.probability
            box_class_id = bbox.id
            if(bbox.class_id == 'car' or bbox.class_id == 'truck' or bbox.class_id == 'bus'):
                out_detections.append(Detection(box=[xmin, ymin, xmax, ymax], score=confidence))
                out_class_id.append(box_class_id)

        return out_detections, out_class_id


    def create_d_msgs_box(self, track):
        one_box = BoundingBox()

        one_box.id = int(track.id[:3], 16)
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

        # print("boxes--------------------")
        # for box_print in boxes.bounding_boxes:
        #     print(box_print)
        # print("\n\n")
        
        self.pub.publish(boxes)

    def process_boxes_ros2(self, msg):
        try:
            
            # detections = self.motpy_detector.process_image(self.frame)

            detections, class_id = self.bboxes2out_detections(msg)

            self.tracker.step(detections)
            tracks = self.tracker.active_tracks(min_steps_alive=3)

            self.publish_d_msgs(tracks, msg)

            # preview the boxes on self.frame----------------------------------------
            # print(detections)
            for det in detections:
                draw_detection(self.frame, det)

            for track in tracks:
                draw_track(self.frame, track)

            cv2.imshow('tracking', self.frame)
            cv2.waitKey(int(1000 * self.dt))

        except Exception as err:
            print(err)

    def process_image_ros2(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")

        except Exception as err:
            print(err)

def ros_main(args = None):
    # os.makedirs(download_path, exist_ok=True)
    rclpy.init(args=args)

    motpy2darknet_class = motpy2darknet()
    rclpy.spin(motpy2darknet_class)

    motpy2darknet_class.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros_main()