import launch
import launch_ros.actions

def generate_launch_description():

    webcam = launch_ros.actions.Node(
        package='v4l2_camera', node_executable='v4l2_camera_node')
    motpy_ros = launch_ros.actions.Node(
        package='motpy_ros', node_executable='face_tracking', output='screen')

    return launch.LaunchDescription([
        webcam,
        motpy_ros,
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=motpy_ros,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
    ])