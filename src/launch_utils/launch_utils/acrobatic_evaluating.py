import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchLogDir
from launch_ros.actions import Node

from python_utils.utils import Name

def generate_launch_description():
    training_params = os.path.join(
        get_package_share_directory('global_utils'),
        'config',
        'acrobatic_evaluating.yaml'
    )

    node_name: str = "acrobatic_oa"
    time_format: str = Name.Dynamic.LOG_FILE_FORMAT().get()
    ts = datetime.now().strftime(time_format)

    return LaunchDescription([
        SetEnvironmentVariable(
            name='ROS_LOG_DIR',
            value=LaunchLogDir()
        ),
        Node(
            package=node_name,
            executable=node_name,
            name=node_name,
            parameters=[training_params],
            arguments=[
                "--ros-args",
                "--log-level", "warn",
                "--disable-stdout-logs",
                "--log-file-name", f"{node_name}_{ts}",
            ],
        )
    ])
