#!/bin/python3
from motpy import Detection, MultiObjectTracker
import rclpy
from rclpy.node import Node

from bboxes_ex_msgs.msg import BoundingBoxes
from bboxes_ex_msgs.msg import BoundingBox

class darknet_tracking(Node):
    def __init__(self) -> None:
        super().__init__('darknet_tracking')

        ## By run() function --------------------------------
        self.model_spec = {'order_pos': 1, 'dim_pos': 2,
                            'order_size': 0, 'dim_size': 2,
                            'q_var_pos': 5000., 'r_var_pos': 0.1}

        self.dt = 1 / 30.0  # assume 15 fps
        self.tracker = MultiObjectTracker(dt=self.dt, model_spec=self.model_spec)
        self.tag = 'face'

        self.pub = self.create_publisher(BoundingBoxes,"tracking_data/bounding_boxes", 10)
        self.sub = self.create_subscription(BoundingBoxes,"bounding_boxes",self.process_boxes_ros2, 10)

    def bboxes2out_detections(self, bboxes:BoundingBoxes):
        out_detections = []
        
        for bbox in bboxes.bounding_boxes:
            out_detections.append(Detection(box=[bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax], score=bbox.probability))
            
        return out_detections

    def create_d_msgs_box(self, track, class_tag:str) -> BoundingBox:
        one_box = BoundingBox()

        one_box.id = int(track.id[:3], 16)
        one_box.class_id = class_tag
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

    def process_boxes_ros2(self, msg:BoundingBoxes) -> None:
        detections = self.bboxes2out_detections(msg)


        self.tracker.step(detections)
        tracks = self.tracker.active_tracks(min_steps_alive=3)

        # print(tracks)
        # print(class_name)

        self.publish_d_msgs(tracks,msg)

def ros_main(args = None):
    # os.makedirs(download_path, exist_ok=True)
    rclpy.init(args=args)

    darknet_tracking_class = darknet_tracking()
    rclpy.spin(darknet_tracking_class)

    darknet_tracking_class.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros_main()