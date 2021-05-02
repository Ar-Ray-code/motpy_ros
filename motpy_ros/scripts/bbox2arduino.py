#!/bin/python3
import rospy
from std_msgs.msg import UInt16
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class servo_angle():
    def __init__(self):
        self.unique_id = 0
        rospy.init_node('servo_angle')

        self.width = rospy.get_param("~frame_size/width", 360)
        self.height = rospy.get_param("~frame_size/height", 240)

        rospy.Subscriber("bounding_boxes",BoundingBoxes,self.process_bbox)
        self.pub = rospy.Publisher("servo", UInt16)

        rospy.spin()

    def process_bbox(self, msg):
        detection_flag = 0
        print("\n----------")
        if(len(msg.bounding_boxes)==0):
            return
        
        for bbox in msg.bounding_boxes:
            if self.unique_id == bbox.id:
                bbox_x = (bbox.xmax+bbox.xmin)//2
                bbox_y = (bbox.ymax+bbox.ymin)//2
                detection_flag = 1
                position_x = int(bbox_x / self.width  * 180)
                position_y = int(bbox_y / self.height * 180)

                print("center_x:",bbox_x / self.width)
                print("position_x:", position_x, " position_y:", position_y)
                self.pub.publish(position_x)

        if (detection_flag == 0) :
            self.unique_id = msg.bounding_boxes[0].id
            print("Find new id")

def ros_main(args = None):
    servo_angle_node = servo_angle()

if __name__ == "__main__":
    ros_main()