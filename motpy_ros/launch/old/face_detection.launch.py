import launch
import launch_ros.actions

def generate_launch_description():

    webcam = launch_ros.actions.Node(
        package='v4l2_camera', node_executable='v4l2_camera_node',
        namespace='camera/color',
    )
    face_detect = launch_ros.actions.Node(
        package='motpy_ros', node_executable='face_detect')

    return launch.LaunchDescription([
        webcam,
        face_detect,
    ])