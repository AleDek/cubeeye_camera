import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# launch camera node and select the camera with [serial number] and run it with [type]
# ex) ros2 launch cubeeye_camera cubeeye_camera_auto_launch.py serialnumber:='nntab005098108' frametype:='6'
def generate_launch_description():
   

    return LaunchDescription([

        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "base_to_cubeye",
            arguments=["0.56", "0.0", "0.3", "0.0", "0.95", "0", "base_link", "cubeeye_link"]
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "cubeye_to_pcl",
            arguments=["0", "0", "0", "0", "0", "3.14", "cubeeye_link", "pcl"]
        ),

        Node(
            package='cubeeye_camera',
            executable='cubeeye_camera_node',
            name='cubeeye_camera_node',
            output='screen',
            parameters=[
                {"autorun_onoff" : True},
                {"autorun_serialnumber" : 'I200DU2608000043'},
                {"autorun_frametype" : 36}
            ]
        )
    ])
