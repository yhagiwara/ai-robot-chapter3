import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

# select_answer.py  speech_recognition.py  speech_synthesis.py

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),

        launch_ros.actions.Node(              
            package="voice_pkg", executable="speech_recognition", output="screen",
            name=['speech_recognition']),

        launch_ros.actions.Node(
            package="voice_pkg", executable="select_answer", output="screen",
            name=['select_answer']),

        launch_ros.actions.Node(              
            package="voice_pkg", executable="speech_synthesis", output="screen",
            name=['speech_synthesis']),
    ])
