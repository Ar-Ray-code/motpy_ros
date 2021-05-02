# motpy_ros
ROS example program using [motpy](https://github.com/wmuron/motpy).

![example](pictures_for_readme/example.jpg)

## Dependencies

- ROS1 Noetic
- OpenCV4
- [darknet_ros_msgs](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros_msgs)
- ros-noetic-usb_cam

## Installation

```shell
# Create workspace
$ source /opt/ros/noetic/setup.bash
$ cd ~
$ mkdir -p ros1_ws/src
$ cd ros1_ws/src

# Clone repository
$ git clone --branch noetic-devel https://github.com/Ar-Ray-code/motpy_ros.git
$ cd motpy_ros
$ pip3 install requirements.txt

$ cd ~/ros1_ws/src
$ catkin_make
```



## Demo 1(example_mosaic)

Connect your webcam (/dev/video0) and execute the following commands.

```shell
$ source /opt/ros/noetic/setup.bash
$ source ~/ros1_ws/devel/setup.bash
$ roslaunch motpy_ros example_mosaic.launch
```

![Rqt_graph_example_mosaic](pictures_for_readme/Rqt_graph_example_mosaic.png)

### The role of each Node is as follows

- camera : Publish the video from the webcam.
- face_detector : "res10_300x300_ssd_iter_140000.caffemodel" (OpenCV) to detect faces.
- darknet_tracking : Tracking boundingboxes by motpy.
- mosaic_bbox : Display tracking image with mosaic.

![tracking](pictures_for_readme/tracking.png)

About topic

`$ rostopic echo /tracking_data/bounding_boxes`

```txt
header: 
  seq: 464
  stamp: 
    secs: 1619954372
    nsecs: 821653734
  frame_id: "head_camera"
image_header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
bounding_boxes: 
  - 
    probability: 0.783602034603571
    xmin: 200
    ymin: 103
    xmax: 272
    ymax: 181
    id: 3711
    Class: "face"
  - 
    probability: 0.8840379995412154
    xmin: 44
    ymin: 111
    xmax: 176
    ymax: 236
    id: 2153
    Class: "face"

```



## Reference

- https://github.com/wmuron/motpy
- https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros_msgs

## About writer

- Ar-Ray : Japanese student. 
- Blog(Japanese) : https://ar-ray.hatenablog.com/
- Twitter : https://twitter.com/Ray255Ar

