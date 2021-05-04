import launch
import launch_ros.actions

def generate_launch_description():

    webcam = launch_ros.actions.Node(
        package='v4l2_camera', node_executable='v4l2_camera_node',
        namespace='camera/color',
    )
    face_detect = launch_ros.actions.Node(
        package='motpy_ros', node_executable='face_detect',
        parameters=[
            {'imshow_isshow' : 0}
        ]
    )

    tracking = launch_ros.actions.Node(
        package='motpy_ros', node_executable='darknet_tracking'
    )

    mosaic = launch_ros.actions.Node(
        package='motpy_ros', node_executable='mosaic_bbox',
        remappings=[
            ('bounding_boxes','/tracking_data/bounding_boxes')
        ],
    )
    rqt_graph = launch_ros.actions.Node(
        package='rqt_graph', node_executable='rqt_graph'
    )

    return launch.LaunchDescription([
        webcam,
        face_detect,
        tracking,
        mosaic,
        rqt_graph
    ])