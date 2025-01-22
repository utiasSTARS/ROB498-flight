import os
from launch_ros.actions import Node
from launch.launch_description_source import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory


def generate_launch_description():
    launch_description = []
    #
    launch_description.append(
        #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("px4_autonomy_modules"), "launch", "mavros.launch.py"
                )
            )
        )
    )
    #
    launch_description.append(
        #
        Node(
            package="px4_autonomy_modules",
            executable="task_2.py",
            name="rob498_drone_00",
            output="screen",
            emulate_tty=True
        )
    )
    return LaunchDescription(launch_description)