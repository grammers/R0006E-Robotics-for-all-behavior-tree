from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
                {"tree_path" : "/home/grammers/colcon_ws/behviorTree/src/behaviorTreerRos/trees/"},
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
