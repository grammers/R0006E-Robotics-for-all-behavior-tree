from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import getpass
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    rfa = get_package_share_directory('behavior_tree_ros')
    user = getpass.getuser()
    tree_path = LaunchConfiguration('tree_path')
    tree_path_arg = DeclareLaunchArgument('tree_path', default_value='/home/ros/assigment/')
    tree_file = LaunchConfiguration('tree_main')
    tree_file_arg = DeclareLaunchArgument('tree_main', default_value='behaviorTreeLab')

    node = Node(
        package="behavior_tree_ros",
        executable="behavior_tree_ros",
        name="shafter",
        namespace="drone",
        output="screen",
        emulate_tty=True,
        parameters=[
            #{"tree_main" : "Hover"},
            {"tree_main" : tree_file},
            #{"tree_main" : "behaviorTreeLab"},
            #{"tree_path" : "/home/" + user + "/colcon_ws/src/R0006E-Robotics-for-all-behavior-tree/behaviorTreeRos/trees/"},
            #{"tree_path" : rfa + '/trees'}, #os.path.join(rfa, 'trees')},
            {"tree_path" : tree_path},
            {"take_off_topic" : "start"},
            {"safety_land_topic" : "safety_land"},
            {"flying_status_topic" : "flying"},
            {"odom_topic" : "/localization"},
            {"wp_topic" : "/clicked_point"},
            {"arming_srvice" : "mavros/cmd/arming"},
            {"mode_service" : "mavros/set_mode"},
            {"BT_tick_time" : 500}
        ]
    )

    ld.add_action(node)
    return LaunchDescription([tree_path_arg, tree_file_arg, node])
