#!/bin/python3
import os
from os.path import expanduser
from typing import Sequence
from urllib.request import urlretrieve

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class mosaic_class:
    def __mosaic(self, src, ratio=0.1):
        small = cv2.resize(src, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST)
        return cv2.resize(small, src.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)

    def mosaic_area(self, src, x, y, width, height, id, ratio=0.1):
        dst = src.copy()
        dst[y:y + height, x:x + width] = self.__mosaic(dst[y:y + height, x:x + width], ratio)
        return dst

class mosaic_bbox(Node):
    def __init__(self):
        super().__init__('mosaic_bbox')

        self.mosaic = mosaic_class()

        self.bridge = CvBridge()
        self.depth_isget = 0

        self.declare_parameter('frame_size/width',360)
        self.declare_parameter('frame_size/height',240)
        self.declare_parameter('imshow_isshow',1)

        self.width = self.get_parameter('frame_size/width').value
        self.height = self.get_parameter('frame_size/height').value
        self.imshow_isshow = self.get_parameter('imshow_isshow').value

        self.sub = self.create_subscription(Image,"motpy/image_raw",self.process_image_ros2, 10)
        self.sub = self.create_subscription(Image,"camera/depth/image_rect_raw",self.process_depth, 10)
        self.sub = self.create_subscription(BoundingBoxes,"bounding_boxes",self.process_bbox, 10)

    def process_bbox(self, msg):
        try:
            mosaiced = self.frame.copy()
            print("\n----------")
            for bbox in msg.bounding_boxes:
                mosaiced = self.mosaic.mosaic_area(mosaiced, bbox.xmin, bbox.ymin, bbox.xmax-bbox.xmin, bbox.ymax-bbox.ymin, bbox.id,ratio=0.05)
                
                print("\n",bbox)
                if self.depth_isget == 1:
                    m_person_depth = self.m_depth_image[(bbox.ymax+bbox.ymin)//2][(bbox.xmax+bbox.xmin)//2]
                    print(m_person_depth)
                else :
                    m_person_depth = -1

                cv2.putText(mosaiced, "id:"+str(bbox.id)+" dist:"+str(m_person_depth)+"mm", (bbox.xmin, bbox.ymin), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 1, cv2.LINE_AA)
            
            if self.imshow_isshow:
                cv2.imshow('mosaic', mosaiced)
                cv2.waitKey(1)
            
        except Exception as err:
            print(err)

    def process_image_ros2(self, msg):
        try:
            self.frame = cv2.resize(self.bridge.imgmsg_to_cv2(msg,"bgr8"),(self.width,self.height))
        except Exception as err:
            print(err)
    
    def process_depth(self,msg):
        self.m_depth_image = cv2.resize(self.bridge.imgmsg_to_cv2(msg, 'passthrough'),(self.width,self.height))
        self.depth_isget = 1

def ros_main(args = None):
    rclpy.init(args=args)

    mosaic_bbox_class = mosaic_bbox()
    rclpy.spin(mosaic_bbox_class)

    mosaic_bbox_class.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros_main()