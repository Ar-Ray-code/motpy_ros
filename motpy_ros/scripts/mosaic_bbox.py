#!/bin/python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class mosaic_class:
    def mosaic(self, src, ratio=0.1):
        small = cv2.resize(src, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST)
        return cv2.resize(small, src.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)

    def mosaic_area(self, src, x, y, width, height, id, ratio=0.1):
        dst = src.copy()
        dst[y:y + height, x:x + width] = self.mosaic(dst[y:y + height, x:x + width], ratio)
        return dst

class mosaic_bbox(mosaic_class):
    def __init__(self):

        self.bridge = CvBridge()
        self.get_depth = 0

        rospy.init_node('mosaic_bbox')

        self.width = rospy.get_param("~tracking_size/width", 360)
        self.height = rospy.get_param("~tracking_size/height", 240)

        rospy.Subscriber("motpy/image_raw",Image,self.process_image_ros1)
        rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.process_depth)
        rospy.Subscriber("/bounding_boxes",BoundingBoxes,self.process_bbox)

        rospy.spin()

    def process_bbox(self, msg):
        try:
            mosaiced = self.frame.copy()
            print("\n----------")
            for bbox in msg.bounding_boxes:
                mosaiced = self.mosaic_area(mosaiced, bbox.xmin, bbox.ymin, bbox.xmax-bbox.xmin, bbox.ymax-bbox.ymin, bbox.id,ratio=0.05)
                
                print("\n",bbox)
                if self.get_depth == 1:
                    m_person_depth = self.m_depth_image[(bbox.ymax+bbox.ymin)//2][(bbox.xmax+bbox.xmin)//2]
                    print(m_person_depth)
                else :
                    m_person_depth = -1

                cv2.putText(mosaiced, "id:"+str(bbox.id)+" dist:"+str(m_person_depth)+"mm", (bbox.xmin, bbox.ymin), cv2.FONT_HERSHEY_PLAIN, 1, (0,0, 255), 1, cv2.LINE_AA)
                
            cv2.imshow('mosaic', mosaiced)
            cv2.waitKey(1)
            
        except Exception as err:
            print(err)

    def process_image_ros1(self, msg):
        self.frame = cv2.resize(self.bridge.imgmsg_to_cv2(msg,"bgr8"),(self.width,self.height))
    
    def process_depth(self,msg):
        self.m_depth_image = cv2.resize(self.bridge.imgmsg_to_cv2(msg, 'passthrough'),(self.width,self.height))
        self.get_depth = 1

def ros_main(args = None):
    mosaic_class = mosaic_bbox()

if __name__ == "__main__":
    ros_main()