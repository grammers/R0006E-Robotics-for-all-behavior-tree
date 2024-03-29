from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rfa = get_package_share_directory('behavior_tree_ros')
    user = os.getlogin()
    return LaunchDescription([
        Node(
            package="behavior_tree_ros",
            executable="behavior_tree_ros",
            name="shafter",
            namespace="drone",
            output="screen",
            emulate_tty=True,
            parameters=[
                #{"tree_main" : "Hover"},
                {"tree_main" : "behaviorTreeLab"},
                {"tree_path" : "/home/" + user + "/colcon_ws/src/R0006E-Robotics-for-all-behavior-tree/behaviorTreeRos/trees/"},
                #{"tree_path" : rfa + '/trees'}, #os.path.join(rfa, 'trees')},
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
    ])
