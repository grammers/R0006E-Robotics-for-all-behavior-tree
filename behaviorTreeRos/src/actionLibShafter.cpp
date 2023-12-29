#include "actionLibShafter.hpp"


WpExist::WpExist(const std::string& name) : BT::ConditionNode(name, {}){}

void WpExist::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus WpExist::tick(){
    if(m_shafter->wp_exist()){
        std::cout<<"WpExist: true wp do exisit"<<std::endl;
        print_once = true;
        return BT::NodeStatus::SUCCESS;
    } else {
        if(print_once){ 
            std::cout<<"WpExist: false wp don't exist"<<std::endl;
            print_once = false;
        }

        return BT::NodeStatus::FAILURE;
    }
}

Flying::Flying(const std::string& name) : BT::ConditionNode(name, {}){}

void Flying::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus Flying::tick(){
    if(m_shafter->get_flying()){
        std::cout<<"Flying: true drone is flying"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout<<"Flying: fals dorn is not flying"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
}


AtWP::AtWP(const std::string& name) : BT::ConditionNode(name, {}){}

void AtWP::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus AtWP::tick(){
    if(m_shafter->at_wp()){
        std::cout<<"at wp"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout<<"not at wp"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
}


FlySafe::FlySafe(const std::string& name) : BT::SyncActionNode(name, {}){}

void FlySafe::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus FlySafe::tick(){
    if(m_shafter->fly_safe()){
        std::cout<<"fly_safe"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout<<"don't fly"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

GoToWp::GoToWp(const std::string& name) : BT::StatefulActionNode(name, {}){}

void GoToWp::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus GoToWp::onStart(){
    std::cout<<"GoToWp:"<<std::endl;
    m_shafter->go_to_wp();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWp::onRunning(){
    if(m_shafter->at_wp()){
        std::cout<<"GoToWp: wp reatched"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToWp::onHalted(){
    std::cout<<"go to wp halted"<<std::endl;
}

PathFollow::PathFollow(const std::string& name) : BT::StatefulActionNode(name, {}){}

void PathFollow::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus PathFollow::onStart(){
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PathFollow::onRunning(){
    return BT::NodeStatus::SUCCESS;
}

void PathFollow::onHalted(){
    std::cout<<"PathFollow halted"<<std::endl;
}

Stabelize::Stabelize(const std::string& name) : BT::StatefulActionNode(name, {}){}

void Stabelize::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus Stabelize::onStart(){
    std::cout<<"Stabelize: set dron mode to Stabelized"<<std::endl;
    if(m_shafter->get_flying()){
        return BT::NodeStatus::FAILURE;
    }
    std::string mode = "STABILIZED";
    m_shafter->set_mode(mode);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Stabelize::onRunning(){
    if(m_shafter->mode_response() == 1){
        std::cout<<"Stabelize: set dron mode successful set to Stabelized"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else if (m_shafter->mode_response() == 0){
        return BT::NodeStatus::RUNNING;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

void Stabelize::onHalted(){
    std::cout<<"Stabelize is halded no behavior implemented"<<std::endl;
}

Disarm::Disarm(const std::string& name) : BT::StatefulActionNode(name, {}){}

void Disarm::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus Disarm::onStart(){
    std::cout<<"Disarm: disarming drone"<<std::endl;
    if(m_shafter->get_flying()){
        return BT::NodeStatus::FAILURE;
    }

    m_shafter->arm_call(false);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Disarm::onRunning(){
    if(!m_shafter->get_arm_state()){
        std::cout<<"Disarm: drone disarmed, safte to aprotch (unles it arms)"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void Disarm::onHalted(){
    std::cout<<"disarm is halted, no logic implementation"<<std::endl;
}

SaftyLand::SaftyLand(const std::string& name) : BT::StatefulActionNode(name, {}){}

void SaftyLand::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus SaftyLand::onStart(){
    std::cout<<"SaftyLand: landing safly"<<std::endl;
    m_shafter->safty_land();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaftyLand::onRunning(){
    bool flying = m_shafter->get_flying();
    if(!flying){
        std::cout<<"SaftyLand: the egal has landed"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::RUNNING;
    }
}

void SaftyLand::onHalted(){
    std::cout<<"Safety land halted, no logic implementation"<<std::endl;
}

Land::Land(const std::string& name) : BT::StatefulActionNode(name, {}){}

void Land::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus Land::onStart(){
    std::cout<<"Land: landing"<<std::endl;
    m_shafter->land();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Land::onRunning(){
    if(m_shafter->get_landing() == 1){
        std::cout<<"Land: the egal has landed"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }else if(m_shafter->get_landing() == 0){
        return BT::NodeStatus::RUNNING;
    }else{
        return BT::NodeStatus::FAILURE;
    }
}

void Land::onHalted(){
    std::cout<<"land halted, no logic implementation"<<std::endl;
}


Arm::Arm(const std::string& name) : BT::StatefulActionNode(name, {}){ 
}

void Arm::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus Arm::onStart(){
    std::cout<<"Arm: comand sent"<<std::endl;
    m_shafter->arm_call(true);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Arm::onRunning(){
    if(m_shafter->get_arm_state()){
        std::cout<<"Arm: successful "<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void Arm::onHalted(){
    std::cout<<"arm halted no logic implementation"<<std::endl;
}

TakeOff::TakeOff(const std::string& name) : BT::StatefulActionNode(name, {}) {}

void TakeOff::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus TakeOff::onStart(){
    m_shafter->take_off();
    std::cout<<"TakeOff: drone is taking off"<<std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeOff::onRunning(){
    if(m_shafter->get_flying()){
        std::cout<<"TakeOff: we have take off"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::RUNNING;
    }
}

void TakeOff::onHalted(){
    std::cout<<"takeOff halted"<<std::endl;
}

OffBoard::OffBoard(const std::string& name) : BT::StatefulActionNode(name, {}) {}

void OffBoard::init(std::shared_ptr<ShafterBT> shafter){
    m_shafter = shafter;
}

BT::NodeStatus OffBoard::onStart(){
    std::cout<<"OffBoard: control mode sent"<<std::endl;
    std::string mode = "OFFBOARD";
    m_shafter->set_mode(mode);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OffBoard::onRunning(){
    if(m_shafter->mode_response() == 1){
        std::cout<<"OffBoard: control mode set to OffBoard"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    } else if (m_shafter->mode_response() == 0){
        return BT::NodeStatus::RUNNING;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

void OffBoard::onHalted(){
    std::cout<<"OffBoard halded no behavior implemented"<<std::endl;
}

///////////////////////////////////////////////////
/////// Shafter /////////////////////////////////
/////////////////////////////////////////////

////// Flight mode, OFFBOARD //////////
int ShafterBT::mode_response(){
    return shafter_mode;
}

void ShafterBT::set_mode(std::string mode){
    shafter_mode = 0;
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode.data();

    auto response = mode_srv->async_send_request(request, std::bind(&ShafterBT::set_mode_response, this, std::placeholders::_1));
}

void ShafterBT::set_mode_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture response){
    if(response.get()->mode_sent){
        shafter_mode = 1;
    } else {
        shafter_mode = -1;
    }
}


////// ARMING //////////// 
void ShafterBT::arm_call(bool set){
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = set;
    auto response = arm_srv->async_send_request(request, std::bind(&ShafterBT::set_arm_responce, this, std::placeholders::_1));
}

void ShafterBT::set_arm_responce(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture responce){
    if(responce.get()->success){
        arm_state = !arm_state; // should be properly tested to set acurate
    } else {
        RCLCPP_INFO_STREAM(get_logger(),"ARMING returend failure");
    }
}

bool ShafterBT::get_arm_state(){
    return arm_state;
}

///// safty land /////
void ShafterBT::safty_land(){
    auto msg = std_msgs::msg::String();    
    safty_land_pub->publish(msg);
}

///// land ////
void ShafterBT::land(){
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = land_srv->async_send_request(request, std::bind(&ShafterBT::set_land_response, this, std::placeholders::_1));
    land_state = 0;
}

void ShafterBT::set_land_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture responce){
    if(responce.get()->success){
        flying = false;
        land_state = 1;
    }
    else{
        land_state = -1;
    }
}

int ShafterBT::get_landing(){
    return land_state;
}

/// flying status ////////
void ShafterBT::take_off(){
    auto msg = std_msgs::msg::String();
    take_off_pub->publish(msg);
}

void ShafterBT::fly_callback(const std_msgs::msg::Bool::SharedPtr msg){
    flying = msg->data;
    std::cout<<"fly callback "<<flying<<std::endl;
}

bool ShafterBT::get_flying(){
    return flying;
}

void ShafterBT::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    odom = *msg;
}

void ShafterBT::wp_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    std::cout<<"new wp is clicked"<<std::endl;
    geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position = msg->point;
    pose.pose.position.z += 1.5;
    wp_list.poses.push_back(pose);
    //std::cout<<wp_list.poses.size()<<std::endl;
}

/// go to wp///
void ShafterBT::go_to_wp(){
    go_to_wp_pub->publish(wp_list.poses[0]);
}

/// atWP ///
bool ShafterBT::at_wp(){
    if (std::sqrt(std::pow(wp_list.poses[0].pose.position.x - odom.pose.pose.position.x, 2) +
        std::pow(wp_list.poses[0].pose.position.y - odom.pose.pose.position.y, 2) +
        std::pow(wp_list.poses[0].pose.position.z - odom.pose.pose.position.z, 2)) < 1.0){


        nav_msgs::msg::Path temp = wp_list;
        wp_list = nav_msgs::msg::Path();
        for (int i = 1; i < int(temp.poses.size()); i++){
            wp_list.poses.push_back(temp.poses[i]);

        }
        //std::cout<<wp_list.poses.size()<<std::endl;
        return true;
    }
    else {
        return false;
    }
}

bool ShafterBT::wp_exist(){
    return wp_list.poses.size() != 0;
}

/// safty checks ///
/// flying ///
bool ShafterBT::fly_safe(){
    double odom_dt = (rclcpp::Clock().now() - odom.header.stamp).nanoseconds();
    if (odom_dt > 500000000){
        return false;
    } 
    return true;
}


///// BT //////////////
std::string ShafterBT::get_tree(){
    std::string main_tree = this->get_parameter("tree_main").get_parameter_value().get<std::string>();
    std::cout<<"Main Tree to execute: "<<main_tree<<std::endl;
    return main_tree;
}

std::string ShafterBT::get_dir(){
    return this->get_parameter("tree_path").get_parameter_value().get<std::string>();
}


void ShafterBT::tick_timer_cb(){

    auto tree_status = tree->tickOnce();
    if(tree_status != BT::NodeStatus::RUNNING){
        std::cout<<tree_status<<": A smal step for humanity, but a huage leap for a robotics enginer!"<<std::endl;
        //rclcpp::shutdown();
    }
}

void ShafterBT::set_tree(std::shared_ptr<BT::Tree> tree){
    this->tree = tree;
}

ShafterBT::ShafterBT() : Node("shafter"){

    this->declare_parameter("tree_main", "main_tree_ID");
    this->declare_parameter("tree_path", "path_to_dir_with_trees");
    this->declare_parameter("take_off_topic", "start");
    this->declare_parameter("safety_land_topic", "safety_land");
    this->declare_parameter("land_service", "land");
    this->declare_parameter("flying_status_topic", "flying");
    this->declare_parameter("arming_srvice", "mavros/cmd/arming");
    this->declare_parameter("mode_service", "mavros/set_mode");
    this->declare_parameter("BT_tick_time",500);
    this->declare_parameter("odom_topic","localization");
    this->declare_parameter("wp_topic","/test/clicked_point");
    this->declare_parameter("go_to_wp_topic","/go_to_wp");
    //std::string treexml = this->get_parameter("tree_main").get_parameter_value().get<std::string>();
    //std::string path = this->get_parameter("tree_path").get_parameter_value().get<std::string>();
    //std::cout<<"shafter init "<<treexml<<std::endl;
    
    take_off_pub = this->create_publisher<std_msgs::msg::String>(this->get_parameter("take_off_topic").get_parameter_value().get<std::string>(), 1);
    safty_land_pub = this->create_publisher<std_msgs::msg::String>(this->get_parameter("safety_land_topic").get_parameter_value().get<std::string>(), 1);
    go_to_wp_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->get_parameter("go_to_wp_topic").get_parameter_value().get<std::string>(), 1);

    fly_sub = this->create_subscription<std_msgs::msg::Bool>(this->get_parameter("flying_status_topic").get_parameter_value().get<std::string>(), 1, std::bind(&ShafterBT::fly_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").get_parameter_value().get<std::string>(), 1, std::bind(&ShafterBT::odom_callback, this, std::placeholders::_1));
    wp_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(this->get_parameter("wp_topic").get_parameter_value().get<std::string>(), 1, std::bind(&ShafterBT::wp_callback, this, std::placeholders::_1));

    arm_srv = this->create_client<mavros_msgs::srv::CommandBool>(this->get_parameter("arming_srvice").get_parameter_value().get<std::string>());
    mode_srv = this->create_client<mavros_msgs::srv::SetMode>(this->get_parameter("mode_service").get_parameter_value().get<std::string>());
    land_srv = this->create_client<std_srvs::srv::Trigger>(this->get_parameter("land_service").get_parameter_value().get<std::string>());

    tick_timer = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("BT_tick_time").get_parameter_value().get<int>()), std::bind(&ShafterBT::tick_timer_cb, this));

    /// temp
    //geometry_msgs::msg::PoseStamped temp = geometry_msgs::msg::PoseStamped();
    //temp.pose.position.z = 2.0;
    //wp_list.poses.push_back(temp);

}


int main(int argc, char * argv[]){
     
    rclcpp::init(argc,argv);
    

    auto shafter = std::make_shared<ShafterBT>();
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Arm>("Arm");
    factory.registerNodeType<TakeOff>("TakeOff");
    factory.registerNodeType<OffBoard>("OffBoard");
    factory.registerNodeType<SaftyLand>("SaftyLand");
    factory.registerNodeType<Land>("Land");
    factory.registerNodeType<Disarm>("Disarm");
    factory.registerNodeType<Stabelize>("Stabelize");
    factory.registerNodeType<FlySafe>("FlySafe");
    factory.registerNodeType<Flying>("Flying");
    factory.registerNodeType<GoToWp>("GoToWp");
    factory.registerNodeType<AtWP>("AtWP");
    factory.registerNodeType<WpExist>("WpExist");

    std::string tree_dir = shafter->get_dir();
    using std::filesystem::directory_iterator;
    for (auto const& file : directory_iterator(tree_dir)){
        if (file.path().extension() == ".xml"){
            factory.registerBehaviorTreeFromFile(file.path().string());
        }
    }
    std::shared_ptr<BT::Tree> tree = std::make_shared<BT::Tree>(factory.createTree(shafter->get_tree()));
    //std::shared_ptr<BT::Tree> tree = std::make_shared<BT::Tree>(factory.createTreeFromFile(shafter->get_tree()));


    for(auto& subTree: tree->subtrees){
        for(auto& node : subTree->nodes){
            if(auto arm = dynamic_cast<Arm*>( node.get())){
                arm->init(shafter);
            }
            else if(auto takeOff = dynamic_cast<TakeOff*>( node.get())){
                takeOff->init(shafter);
            }
            else if(auto offBoard = dynamic_cast<OffBoard*>( node.get())){
                offBoard->init(shafter);
            }
            else if(auto safteyLand = dynamic_cast<SaftyLand*>( node.get())){
                safteyLand->init(shafter);
            }
            else if(auto land = dynamic_cast<Land*>( node.get())){
                land->init(shafter);
            }
            else if(auto disarm = dynamic_cast<Disarm*>( node.get())){
                disarm->init(shafter);
            }
            else if(auto stabelize = dynamic_cast<Stabelize*>( node.get())){
                stabelize->init(shafter);
            }
            else if(auto flySafe = dynamic_cast<FlySafe*>( node.get())){
                flySafe->init(shafter);
            }
            else if(auto flying = dynamic_cast<Flying*>( node.get())){
                flying->init(shafter);
            }
            else if(auto goToWp = dynamic_cast<GoToWp*>( node.get())){
                goToWp->init(shafter);
            }
            else if(auto atWp = dynamic_cast<AtWP*>( node.get())){
                atWp->init(shafter);
            }
            else if(auto wpExist = dynamic_cast<WpExist*>( node.get())){
                wpExist->init(shafter);
            }
        }

    }

    shafter->set_tree(tree);

    rclcpp::spin(shafter);
    rclcpp::shutdown();

    return 0;

}
