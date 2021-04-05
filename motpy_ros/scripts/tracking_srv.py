#!/bin/python3

import cv2
import rospy

from motpy import Detection, MultiObjectTracker, NpImage, Box, Track
from motpy.core import setup_logger
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from motpy_ros.srv import motpy_bbox, motpy_bboxResponse

class motpy_srv:
    def __init__(self):
        self.model_spec = {'order_pos': 1, 'dim_pos': 2,
                            'order_size': 0, 'dim_size': 2,
                            'q_var_pos': 5000., 'r_var_pos': 0.1}
        self.dt = 1 / 60.0  # assume 15 fps
        self.tracker = MultiObjectTracker(dt=self.dt, model_spec=self.model_spec)

        service = rospy.Service('detect2tracking',motpy_bbox,self.srv_bboxes)
        rospy.spin()

    def bboxes2out_detections(self, bboxes):
        out_detections = []
        for bbox in bboxes.bounding_boxes:
            out_detections.append(Detection(box=[bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax], score=bbox.probability))
        return out_detections

    def create_d_msgs_box(self, track, box_class):
        one_box = BoundingBox()

        one_box.id = int(track.id[:3], 16)
        one_box.Class = box_class
        one_box.probability = float(track.score)
        one_box.xmin = int(track.box[0])
        one_box.ymin = int(track.box[1])
        one_box.xmax = int(track.box[2])
        one_box.ymax = int(track.box[3])

        return one_box

    def return_detect(self, tracks, bboxes_msg):
        i = 0
        
        boxes = BoundingBoxes()
        boxes.header = bboxes_msg.header

        for track in tracks:
            boxes.bounding_boxes.append(self.create_d_msgs_box(track, bboxes_msg.bounding_boxes[i].Class))
            i = i + 1
        return boxes
        

    def srv_bboxes(self, msg):
        darknet_data = BoundingBoxes()
        darknet_data = msg.input_bboxes
        # darknet2motpy
        detections =self.bboxes2out_detections(darknet_data)

        # motpy process
        self.tracker.step(detections)
        tracks = self.tracker.active_tracks(min_steps_alive=3)
        # return 
        send_boxes = self.return_detect(tracks, msg.input_bboxes)
        return motpy_bboxResponse(send_boxes)

def rospy_init(args = None):
    rospy.init_node('motpy_ros_srv',argv=args)
    motpy_srv()

if __name__=="__main__":
    rospy_init()
