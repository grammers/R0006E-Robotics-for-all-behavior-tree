#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>
#include <filesystem>
#include <list>

#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <future>


class ShafterBT : public rclcpp::Node
{
public:
    ShafterBT();

    std::string get_tree();
    std::string get_dir();

    void set_tree(std::shared_ptr<BT::Tree> tree);

    void arm_call(bool set);
    void set_arm_responce(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture response);
    bool get_arm_state();
    
    void set_mode(std::string mode);
    int mode_response();
    void set_mode_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture response);
    void set_land_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response);

    void safty_land();
    void land();

    void take_off();
    bool get_flying();
    int get_landing();

    bool fly_safe();
    void go_to_wp();
    bool at_wp();
    bool wp_reatced();
    bool wp_exist();


    

private:
    int db_tick = 0;

    // action related
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_srv;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_srv;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_srv;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fly_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wp_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr take_off_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safty_land_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr land_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr go_to_wp_pub;
    
    void fly_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void wp_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // behavior tree tick
    rclcpp::TimerBase::SharedPtr tick_timer;
    void tick_timer_cb();

    std::shared_ptr<BT::Tree> tree;

    // state params
    bool arm_state = false;
    bool flying = false;
    int land_state = 0;
    int shafter_mode = 0; // 1 == offbord

    nav_msgs::msg::Odometry odom = nav_msgs::msg::Odometry();
    nav_msgs::msg::Path wp_list = nav_msgs::msg::Path();
    geometry_msgs::msg::Point last_wp = geometry_msgs::msg::Point();

};

class WpExist : public BT::ConditionNode
{
public:
    WpExist(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
    bool print_once = true;
};

class AtWP : public BT::ConditionNode
{
public:
    AtWP(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class Flying : public BT::ConditionNode
{
public:
    Flying(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class GoToWp : public BT::StatefulActionNode
{
public:
    GoToWp(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class FlySafe : public BT::SyncActionNode
{
public:
    FlySafe(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};


class PathFollow : public BT::StatefulActionNode
{
public:
    PathFollow(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class Disarm : public BT::StatefulActionNode
{
public:
    Disarm(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class Stabelize : public BT::StatefulActionNode
{
public:
    Stabelize(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class SaftyLand : public BT::StatefulActionNode
{
public:
    SaftyLand(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class Land : public BT::StatefulActionNode
{
public:
    Land(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};


class Arm : public BT::StatefulActionNode
{
public:
    //Arm();
    Arm(const std::string& name);
    void init(std::shared_ptr<ShafterBT> shafter);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

class TakeOff : public BT::StatefulActionNode
{
public:
    TakeOff(const std::string& name);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    void init(std::shared_ptr<ShafterBT> shafter);

private:
    std::shared_ptr<ShafterBT> m_shafter;
};


class OffBoard : public BT::StatefulActionNode
{
public:
    OffBoard(const std::string& name);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    void init(std::shared_ptr<ShafterBT> shafter);

private:
    std::shared_ptr<ShafterBT> m_shafter;
};

