from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #pkg_name = 'robotics_for_all_BT_sim'
    #pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
    #colcon_cd %s && pwd"' % pkg_name).read().strip()

    rfa = get_package_share_directory('robotics_for_all_BT_sim')
    return LaunchDescription([
        Node(
            package='rviz2',
            #namespace='',
            executable='rviz2',
            #name='rviz2',
            arguments=['-d', [os.path.join(rfa, 'config', 'config_file.rviz')]]
        )
    ])
