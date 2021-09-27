import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    webcam = launch_ros.actions.Node(
        package='v4l2_camera', executable='v4l2_camera_node',
        namespace='camera/color',
        parameters=[
            {"image_size": [640,480]},
        ],
    )

    darknet_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('darknet_ros') , '/launch/darknet_ros.launch.py']),
    )

    tracking = launch_ros.actions.Node(
        package='motpy_ros', executable='darknet_tracking'
    )

    mosaic = launch_ros.actions.Node(
        package='motpy_ros', executable='mosaic_bbox',
        remappings=[
            ('bounding_boxes','/tracking_data/bounding_boxes'),
            ('/motpy/image_raw','/camera/color/image_raw')
        ],
        parameters=[
            {"frame_size/width" : 640},
            {"frame_size/height" : 480},
        ]
    )
    rqt_graph = launch_ros.actions.Node(
        package='rqt_graph', executable='rqt_graph'
    )

    return launch.LaunchDescription([
        webcam,
        darknet_ros,
        tracking,
        mosaic,
        rqt_graph
    ])