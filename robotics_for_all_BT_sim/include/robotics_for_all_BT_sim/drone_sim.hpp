#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "dsp_interfaces/srv/path_cost.hpp"


class DroneSim : public rclcpp::Node {

public:
    DroneSim();

private:
    void arming(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request, std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response);
    void mode(const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request, std::shared_ptr<mavros_msgs::srv::SetMode::Response> response);
    void land(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void start_callback(const std_msgs::msg::String::SharedPtr msg);
    void safety_land_callback(const std_msgs::msg::String::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_responce(rclcpp::Client<dsp_interfaces::srv::PathCost>::SharedFuture responce);
    void timer_callback();

    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr mav_command_srv;
    rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr mav_mode_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv;

    rclcpp::Client<dsp_interfaces::srv::PathCost>::SharedPtr path_srv;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr take_of_msg;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_land_msg;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_msg;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fly_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    rclcpp::TimerBase::SharedPtr tick_timer;

    nav_msgs::msg::Odometry odom = nav_msgs::msg::Odometry();
    nav_msgs::msg::Path path = nav_msgs::msg::Path();
    int index = 0;



};
