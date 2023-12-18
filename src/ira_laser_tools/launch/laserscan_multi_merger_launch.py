from launch import LaunchDescription
from launch_ros.actions import Node
import math
import yaml
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Specify name of param file that should be loaded
    laserscan_multimerger_cmd = Node(package='ira_laser_tools',
                                     executable='laserscan_multi_merger',
                                     name='laserscan_multi_merger_node',
                                     parameters=[
                                         {'destination_frame': "odom"},
                                         {'scan_destination_topic': "/merged_scan"},
                                         {'laserscan_topics': "/lidar_front/scan "
                                                              "/lidar_left/scan "
                                                              "/lidar_right/scan "
                                          },
                                         {'angle_min': -1.57079632679},
                                         {'angle_max': 1.57079632679},
                                         {'angle_increment': 0.01163552834},
                                         {'scan_time': 0.1},
                                         {'range_min': 0.1},
                                         {'range_max': 10.0},
                                         {'use_inf': False},
                                         {'allow_scan_delay': True},
                                         {'max_delay_scan_time': 0.05},
                                         {'max_completion_time': 0.1},
                                         {'max_merge_time_diff': 0.1},
                                         {'best_effort': True},
                                     ],
                                     )

    return LaunchDescription([
        laserscan_multimerger_cmd,
    ])
