import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    launch_launch_share_dir = get_package_share_directory("realsense2_camera")
    realsense = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_launch_share_dir + "/launch/rs_launch.py"]),
    )

    face_detect = launch_ros.actions.Node(
        package='motpy_ros', executable='face_detect',
        parameters=[
            {'imshow_isshow' : 0}
        ]
    )

    tracking = launch_ros.actions.Node(
        package='motpy_ros', executable='darknet_tracking'
    )

    mosaic = launch_ros.actions.Node(
        package='motpy_ros', executable='mosaic_bbox',
        remappings=[
            ('bounding_boxes','/tracking_data/bounding_boxes')
        ],
    )
    rqt_graph = launch_ros.actions.Node(
        package='rqt_graph', executable='rqt_graph'
    )

    return launch.LaunchDescription([
        realsense,
        face_detect,
        tracking,
        mosaic,
        rqt_graph
    ])