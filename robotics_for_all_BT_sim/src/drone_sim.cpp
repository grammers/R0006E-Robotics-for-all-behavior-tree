#include "robotics_for_all_BT_sim/drone_sim.hpp"

DroneSim::DroneSim() : Node("drone_sim"){

    mav_command_srv = this->create_service<mavros_msgs::srv::CommandBool>("drone/mavros/cmd/arming",
        std::bind(&DroneSim::arming, this, std::placeholders::_1, std::placeholders::_2)); 
    mav_mode_srv = this->create_service<mavros_msgs::srv::SetMode>("drone/mavros/set_mode",
        std::bind(&DroneSim::mode, this, std::placeholders::_1, std::placeholders::_2)); 
    land_srv = this->create_service<std_srvs::srv::Trigger>("drone/land",
        std::bind(&DroneSim::land, this, std::placeholders::_1, std::placeholders::_2)); 

    path_srv = this->create_client<dsp_interfaces::srv::PathCost>("/dsp/path_cost");

    take_of_msg = this->create_subscription<std_msgs::msg::String>("drone/start", 1, 
        std::bind(&DroneSim::start_callback, this, std::placeholders::_1));
    safety_land_msg = this->create_subscription<std_msgs::msg::String>("drone/safety_land", 1, 
        std::bind(&DroneSim::safety_land_callback, this, std::placeholders::_1));
    pose_msg = this->create_subscription<geometry_msgs::msg::PoseStamped>("/go_to_wp", 1, 
        std::bind(&DroneSim::pose_callback, this, std::placeholders::_1));

    fly_pub = this->create_publisher<std_msgs::msg::Bool>("drone/flying", 1);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 1);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 1);

tick_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&DroneSim::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Drone sim booted!");

    odom.header.frame_id = "map";
    odom_pub->publish(odom);
}

void DroneSim::timer_callback(){

    if (int(path.poses.size()) > index){
        //std::cout<<index<<std::endl;
        odom.pose.pose = path.poses[index].pose;
        odom_pub->publish(odom);
        index++;
    }

}

void DroneSim::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "wp resived");
    auto request = std::make_shared<dsp_interfaces::srv::PathCost::Request>();
    request->start = odom.pose.pose.position;
    request->stop = msg->pose.position;
    auto response = path_srv->async_send_request(request, std::bind(&DroneSim::path_responce, this, std::placeholders::_1));
}

void DroneSim::path_responce(rclcpp::Client<dsp_interfaces::srv::PathCost>::SharedFuture responce){
    RCLCPP_INFO(this->get_logger(), "Path resived");
    this->path = responce.get()->path;
    //RCLCPP_INFO(this->get_logger(), "Path resived %li", path.poses.size());
    path_pub->publish(path);

    index = 0;
}

void DroneSim::arming(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request, std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response){

    if (request->value == true){
        response->success = true;
        response->result = 1;
        RCLCPP_INFO(this->get_logger(), "Armed");
    }
    else if (request->value == false){
        response->success = true;
        response->result = 0;
        RCLCPP_INFO(this->get_logger(), "Disarmed");
    }
    else{
        response->success = false;
        response->result = 0;
        RCLCPP_INFO(this->get_logger(), "Arming Faild");
    }
}

void DroneSim::mode(const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request, std::shared_ptr<mavros_msgs::srv::SetMode::Response> response){
    if (request->custom_mode == "OFFBOARD"){
        response->mode_sent = true;
        RCLCPP_INFO(this->get_logger(), "OFFBOARD");
    }
    else if (request->custom_mode == "STABILIZED"){
        response->mode_sent = true;
        RCLCPP_INFO(this->get_logger(), "STABILIZED");
    }
    else{
        response->mode_sent = false;
        RCLCPP_INFO(this->get_logger(), "Mode Failed");
    }

}

void DroneSim::land(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::cout<<request<<std::endl;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Landing");
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);

    std_msgs::msg::Bool s;
    s.data = false;
    fly_pub->publish(s);
}

void DroneSim::start_callback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Take off");
    std::cout<<msg->data<<std::endl;
    std_msgs::msg::Bool response;
    response.data = true;
    
    odom.pose.pose.position.z += 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z += 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z += 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z += 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z += 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z += 0.25;
    odom_pub->publish(odom);

    fly_pub->publish(response);
}

void DroneSim::safety_land_callback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Safety land");
    std::cout<<msg->data<<std::endl;
    std_msgs::msg::Bool response;
    response.data = false;
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    odom.pose.pose.position.z -= 0.25;
    odom_pub->publish(odom);
    fly_pub->publish(response);
}

int main (int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSim>());
    rclcpp::shutdown();
    return 0;
}
