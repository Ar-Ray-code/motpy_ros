#!/bin/python3

from motpy import Detection, MultiObjectTracker
import rospy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class darknet_tracking:
    def __init__(self) -> None:

        ## By run() function --------------------------------
        self.model_spec = {'order_pos': 1, 'dim_pos': 2,
                            'order_size': 0, 'dim_size': 2,
                            'q_var_pos': 5000., 'r_var_pos': 0.1}

        self.dt = 1 / 30.0  # assume 15 fps
        self.tracker = MultiObjectTracker(dt=self.dt, model_spec=self.model_spec)

        self.tag = 'face'

        rospy.init_node('darknet_tracking')

        self.pub = rospy.Publisher("tracking_data/bounding_boxes", BoundingBoxes, queue_size=1)
        rospy.Subscriber("bounding_boxes",BoundingBoxes,self.process_boxes_ros1)

        rospy.spin()

    def bboxes2out_detections(self, bboxes:BoundingBoxes):
        out_detections = []
        
        for bbox in bboxes.bounding_boxes:
            out_detections.append(Detection(box=[bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax], score=bbox.probability))
            
        return out_detections

    def create_d_msgs_box(self, track, class_tag:str) -> BoundingBox:
        one_box = BoundingBox()

        one_box.id = int(track.id[:3], 16)
        one_box.Class = class_tag
        one_box.probability = float(track.score)
        one_box.xmin = int(track.box[0])
        one_box.ymin = int(track.box[1])
        one_box.xmax = int(track.box[2])
        one_box.ymax = int(track.box[3])

        return one_box

    def publish_d_msgs(self, tracks, boxes_msg:BoundingBoxes) -> None:
        
        boxes = BoundingBoxes()
        boxes.header = boxes_msg.header
        i = 0
        if(len(tracks)==0):
            self.pub.publish(boxes)
            return
        
        for track in tracks:
            boxes.bounding_boxes.append(self.create_d_msgs_box(track, self.tag))

        self.pub.publish(boxes)

    def process_boxes_ros1(self, msg:BoundingBoxes) -> None:
        detections = self.bboxes2out_detections(msg)


        self.tracker.step(detections)
        tracks = self.tracker.active_tracks(min_steps_alive=3)

        # print(tracks)
        # print(class_name)

        self.publish_d_msgs(tracks,msg)

def ros_main(args = None) -> None:
    darknet_tracking_class = darknet_tracking()

if __name__ == "__main__":
    ros_main()