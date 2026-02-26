#include "pagv_amr_core/amr_core_node.hpp"
#include <cmath>

namespace pagv_amr_core 
{

static double quaternion_to_yaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

AMRCoreNode::AMRCoreNode() : Node("amr_core_node")
{
    RCLCPP_INFO(this->get_logger(), "[AMRCore] Initializing...");

    // Parameters
    this->declare_parameter("agv_id", "0000");
    this->declare_parameter("mqtt_broker", "tcp://localhost:1883");
    this->declare_parameter("control_rate", 20.0);
    this->declare_parameter("state_publish_rate", 1.0);
    this->declare_parameter("vis_publish_rate", 20.0);
    this->declare_parameter("battery_initial_soc", 100.0);
    this->declare_parameter("battery_capacity_wh", 5000.0);
    this->declare_parameter("battery_voltage", 48.0);
    this->declare_parameter("wheel_base", 6.5);
    
    agv_id_ = this->get_parameter("agv_id").as_string();
    mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
    control_rate_ = this->get_parameter("control_rate").as_double();
    state_publish_rate_ = this->get_parameter("state_publish_rate").as_double();
    vis_publish_rate_ = this->get_parameter("vis_publish_rate").as_double();
    battery_soc_ = this->get_parameter("battery_initial_soc").as_double();
    battery_voltage_ = this->get_parameter("battery_voltage").as_double();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    
    // AmrConfig 생성
    AmrConfig config;
    config.speedup_ratio = 1.0;
    config.control_period = 1.0 / control_rate_;
    config.mqtt.server_address = mqtt_broker_;
    config.mqtt.state_publish_period = 1.0 / state_publish_rate_;
    config.mqtt.visualization_publish_period = 1.0 / vis_publish_rate_;
    
    // VDA5050 Protocol 초기화
    vda5050_protocol_ = std::make_unique<Vda5050Protocol>(config);
    vda5050_protocol_->setAgvId(agv_id_);
    vda5050_protocol_->useDefaultConfig(mqtt_broker_);
    vda5050_protocol_->setAmr(this);

    // logging
    const std::string log_dir  = "logs";
    std::string log_path = log_dir + "/vda5050_" + agv_id_ + ".log";
    vda5050_protocol_->enableLogging(log_path);
    vda5050_protocol_->start();
    
    // Order & Navigation
    order_manager_ = std::make_shared<OrderExecutionManager>();
    nav_manager_ = std::make_shared<NavigationManager>();

    // BehaviorTree initialization
    init_behavior_tree();
    
    // ROS2 Publishers/Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    amr_status_pub_ = this->create_publisher<std_msgs::msg::String>("/amr/status", 10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&AMRCoreNode::odom_callback, this, std::placeholders::_1));
    
    // Timers
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
        std::bind(&AMRCoreNode::control_loop, this));
    
    vda5050_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / state_publish_rate_)),
        std::bind(&AMRCoreNode::vda5050_state_publish_loop, this));
    
    vda5050_vis_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / vis_publish_rate_)),
        std::bind(&AMRCoreNode::vda5050_vis_publish_loop, this));
    
    system_initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), "[AMRCore] Ready");
    RCLCPP_INFO(this->get_logger(), "  - AGV ID: %s", agv_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Control rate: %.1f Hz", control_rate_);
    RCLCPP_INFO(this->get_logger(), "  - MQTT broker: %s", mqtt_broker_.c_str());
}

AMRCoreNode::~AMRCoreNode()
{
    if (vda5050_protocol_) 
    {
        vda5050_protocol_->stop();
    }
}

void AMRCoreNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    
    current_pose_.theta = quaternion_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    
    current_pose_.linear_velocity = msg->twist.twist.linear.x;
    current_pose_.angular_velocity = msg->twist.twist.angular.z;
    
    if (!pose_initialized_) {
        pose_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "[AMRCore] Odometry initialized");
    }
    
    // update pose for navigation_manager
    Pose nav_pose;
    nav_pose.x = current_pose_.x;
    nav_pose.y = current_pose_.y;
    nav_pose.theta = current_pose_.theta;
    nav_manager_->set_current_pose(nav_pose);
}

// setOrder
void AMRCoreNode::setOrder(const std::vector<NodeInfo>& nodes, 
                           const std::vector<EdgeInfo>& edges, 
                           const std::vector<NodeInfo>& all_nodes)
{
    RCLCPP_INFO(this->get_logger(), "[AMRCore] setOrder called - NEW ORDER");
    
    nodes_ = nodes;
    edges_ = edges;
    all_nodes_ = all_nodes;
    cur_idx_ = 0;
    cur_edge_idx_ = 0;
    is_angle_adjusting_ = false;
    // wheel_base_ = wheel_base;
    
    completed_nodes_.clear();
    completed_edges_.clear();
    executing_actions_.clear();
    
    if (!nodes_.empty()) {
        completed_nodes_.push_back(nodes_.front());
        RCLCPP_INFO(this->get_logger(), 
                   "[AMRCore] Start node added to completed: %s", 
                   nodes_.front().nodeId.c_str());
    }
    
    Order order;
    order.order_id = "order_" + std::to_string(rclcpp::Clock().now().nanoseconds());
    order.nodes = nodes;
    order_manager_->set_order(order);
    
    if (!edges_.empty()) {
        RCLCPP_INFO(this->get_logger(), 
                   "[AMRCore] Starting order with edge: %s", 
                   edges_[cur_edge_idx_].edgeId.c_str());
        RCLCPP_INFO(this->get_logger(), 
                   "[AMRCore] Nodes: %zu, All nodes: %zu", 
                   nodes_.size(), all_nodes_.size());
        
        const EdgeInfo& edge = edges_[cur_edge_idx_];
        setNavigationTargetFromEdge(edge, nodes_, all_nodes_);
        
        is_paused_ = false;
    }
    
    mission_active_ = true;
    mission_complete_ = false;
    
    if (blackboard_) {
        blackboard_->set("mission_active", true);
        blackboard_->set("mission_complete", false);
    }
}

void AMRCoreNode::updateOrder(const std::vector<NodeInfo>& nodes, 
                              const std::vector<EdgeInfo>& edges,
                              const std::vector<NodeInfo>& all_nodes)
{
    RCLCPP_INFO(this->get_logger(), "[AMRCore] updateOrder called - ORDER UPDATE");
    setOrder(nodes, edges, all_nodes);
}

void AMRCoreNode::setNavigationTargetFromEdge(const EdgeInfo& edge,
    const std::vector<NodeInfo>& nodes,
    const std::vector<NodeInfo>& all_nodes)
{
    // 1. End Node 찾기 (nodes에서)
    const NodeInfo* end_node = nullptr;
    auto it = std::find_if(nodes.begin(), nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.endNodeId; });
    if (it != nodes.end()) {
        end_node = &(*it);
    }
    
    if (!end_node) {
        RCLCPP_ERROR(this->get_logger(), 
                    "[AMRCore] End node not found: %s", edge.endNodeId.c_str());
        return;
    }
    
    // 2. Start Node 찾기 (all_nodes에서)
    const NodeInfo* start_node = nullptr;
    auto sit = std::find_if(all_nodes.begin(), all_nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.startNodeId; });
    if (sit != all_nodes.end()) {
        start_node = &(*sit);
    }
    
    // 3. 현재 위치 (vcu_->getEstimatedPose 대체)
    double cur_x = current_pose_.x;
    double cur_y = current_pose_.y;
    double cur_theta = current_pose_.theta;
    
    // hasTurnCenter에 따라 직선/곡선 경로 설정
    if (edge.hasTurnCenter) 
    {
        //Curved Edge
        RCLCPP_INFO(this->get_logger(), "[AMRCore] Curved edge: %s", edge.edgeId.c_str());
        RCLCPP_INFO(this->get_logger(), "  turnCenter: (%.2f, %.2f)", 
                   edge.turnCenter.x, edge.turnCenter.y);
        
        nav_manager_->set_arc_goal(
            start_node ? start_node->x : cur_x,
            start_node ? start_node->y : cur_y,
            cur_theta,
            end_node->x,
            end_node->y,
            edge.turnCenter.x,
            edge.turnCenter.y);
    } 
    else 
    {
        //Straight Edge
        RCLCPP_INFO(this->get_logger(), "[AMRCore] Straight edge: %s", edge.edgeId.c_str());
        
        NodePosition goal;
        goal.x = end_node->x;
        goal.y = end_node->y;
        goal.theta = end_node->theta;
        
        nav_manager_->set_goal(goal);
    }
    
    RCLCPP_INFO(this->get_logger(), 
               "[AMRCore] Navigation target set: %s at (%.2f, %.2f)",
               end_node->nodeId.c_str(), end_node->x, end_node->y);
}

std::string AMRCoreNode::getState() const
{
    if (mission_complete_) return "IDLE";
    if (mission_active_) return "MOVING";
    return "IDLE";
}

std::vector<NodeInfo> AMRCoreNode::getCurrentNodes() const
{
    std::vector<NodeInfo> current;
    if (cur_idx_ < nodes_.size()) {
        current.push_back(nodes_[cur_idx_]);
    }
    return current;
}

std::vector<EdgeInfo> AMRCoreNode::getCurrentEdges() const
{
    std::vector<EdgeInfo> current;
    if (cur_edge_idx_ < edges_.size()) {
        current.push_back(edges_[cur_edge_idx_]);
    }
    return current;
}

std::string AMRCoreNode::getLastNodeId() const
{
    if (!completed_nodes_.empty()) {
        return completed_nodes_.back().nodeId;
    }
    return "";
}

int AMRCoreNode::getLastNodeSequenceId() const
{
    if (!completed_nodes_.empty()) {
        return completed_nodes_.back().sequenceId;
    }
    return -1;
}

void AMRCoreNode::markNodeAsCompleted(const NodeInfo& node)
{
    completed_nodes_.push_back(node);
    needs_immediate_state_publish_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
               "[AMRCore] Node completed: %s", node.nodeId.c_str());
}

void AMRCoreNode::cancelOrder()
{
    nodes_.clear();
    edges_.clear();
    all_nodes_.clear();
    completed_nodes_.clear();
    completed_edges_.clear();
    executing_actions_.clear();
    
    cur_idx_ = 0;
    cur_edge_idx_ = 0;
    
    mission_active_ = false;
    mission_complete_ = true;
    
    RCLCPP_INFO(this->get_logger(), "[AMRCore] Order cancelled");
}

bool AMRCoreNode::isDriving() const
{
    return mission_active_ && !mission_complete_ && 
           std::abs(current_pose_.linear_velocity) > 0.01;
}

void AMRCoreNode::executeAction(const ActionInfo& action)
{
    RCLCPP_INFO(this->get_logger(), 
               "[AMRCore] Executing action: %s (ID: %s)",
               action.actionType.c_str(), action.actionId.c_str());
}

void AMRCoreNode::step(double dt)
{
    (void)dt;
}

void AMRCoreNode::updateBattery(double dt, bool is_charging)
{
    if (is_charging) {
        battery_soc_ = std::min(100.0, battery_soc_ + dt * 10.0);
    } else if (isDriving()) {
        battery_soc_ = std::max(0.0, battery_soc_ - dt * 0.5);
    }
    
    battery_voltage_ = 43.0 + (battery_soc_ / 100.0) * 5.0;
}

void AMRCoreNode::init_behavior_tree()
{
    blackboard_ = BT::Blackboard::create();
    register_bt_nodes();
    
    std::string pkg_share = ament_index_cpp::get_package_share_directory("pagv_amr_core");
    std::string xml_path = pkg_share + "/behavior_trees/pagv_behavior_tree.xml";
    
    tree_ = factory_.createTreeFromFile(xml_path, blackboard_);
    
    RCLCPP_INFO(this->get_logger(), "[AMRCore] BehaviorTree initialized");
}

void AMRCoreNode::register_bt_nodes()
{
    pagv_amr_core::RegisterModeLayerNodes(factory_);
    pagv_amr_core::RegisterSafetyLayerNodes(factory_);
    pagv_amr_core::RegisterSystemLayerNodes(factory_);
    pagv_amr_core::RegisterMissionLayerNodes(factory_);
    pagv_amr_core::RegisterNavigationLayerNodes(factory_);
}

void AMRCoreNode::update_blackboard()
{
    if (!blackboard_) return;
    
    // // BT → 클래스 방향 동기화
    try {
        auto bt_mission_active = blackboard_->get<bool>("mission_active");
        if (bt_mission_active != mission_active_) {
            RCLCPP_INFO(this->get_logger(), 
                       "[AMRCore] BT changed mission_active: %d -> %d",
                       mission_active_, bt_mission_active);
            mission_active_ = bt_mission_active;
        }
    } catch (...) {}

    blackboard_->set("mode", mode_);
    blackboard_->set("ncu_status", ncu_status_);
    blackboard_->set("emergency_active", emergency_active_);
    blackboard_->set("fault_detected", fault_detected_);
    blackboard_->set("collision_risk", collision_risk_);
    blackboard_->set("battery_soc", battery_soc_);
    blackboard_->set("system_initialized", system_initialized_);
    blackboard_->set("mission_active", mission_active_);
    blackboard_->set("mission_complete", mission_complete_);
    blackboard_->set("goal_reached", goal_reached_);
    blackboard_->set("localization_quality", localization_quality_);
    blackboard_->set("current_x", current_pose_.x);
    blackboard_->set("current_y", current_pose_.y);
    blackboard_->set("current_theta", current_pose_.theta);
}

void AMRCoreNode::control_loop()
{
    if (!pose_initialized_) 
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[AMRCore] Waiting for odometry...");
        return;
    }
    
    const double dt = 1.0 / control_rate_;
    bool is_charging = false;
    updateBattery(dt, is_charging);
    
    update_blackboard();
    
    BT::NodeStatus status = tree_.tickExactlyOnce();
    (void)status; 

    update_blackboard();    
    
    if (mission_active_ && !mission_complete_ && !is_paused_) 
    {
        auto cmd_vel = nav_manager_->compute_velocity_command();
        
        if (max_speed_override_ > 0.0) 
        {
            cmd_vel.linear.x = std::min(cmd_vel.linear.x, max_speed_override_);
        }
        
        cmd_vel_pub_->publish(cmd_vel);
        
        if (nav_manager_->is_goal_reached()) 
        {
            goal_reached_ = true;
            
            if (cur_edge_idx_ < edges_.size()) 
            {
                completed_edges_.push_back(edges_[cur_edge_idx_]);
                cur_edge_idx_++;

                RCLCPP_INFO(this->get_logger(), 
                           "[AMRCore] Edge completed: %s",
                           edges_[cur_edge_idx_ - 1].edgeId.c_str());                
            }
            
            if (cur_idx_ < nodes_.size()) 
            {
                markNodeAsCompleted(nodes_[cur_idx_]);
                cur_idx_++;
            }
            
            if (cur_edge_idx_ < edges_.size()) 
            {
                const EdgeInfo& next_edge = edges_[cur_edge_idx_];
                // setNavigationTargetFromEdge(next_edge);
                setNavigationTargetFromEdge(next_edge, nodes_, all_nodes_);
                goal_reached_ = false;
                
                RCLCPP_INFO(this->get_logger(), 
                           "[AMRCore] Moving to next edge: %s",
                           next_edge.edgeId.c_str());
            } 
            else 
            {
                mission_complete_ = true;
                mission_active_ = false;
                
                RCLCPP_INFO(this->get_logger(), "  - Completed nodes: %zu", completed_nodes_.size());
                RCLCPP_INFO(this->get_logger(), "  - Completed edges: %zu", completed_edges_.size());
            }
        }
    }
    else 
    {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
    }
}

void AMRCoreNode::vda5050_state_publish_loop()
{
    if (!pose_initialized_) return;
    // vda5050_protocol_->publishStateMessage(this);
}

void AMRCoreNode::vda5050_vis_publish_loop()
{
    if (!pose_initialized_) return;
    // vda5050_protocol_->publishVisualizationMessage(this);
}

} // namespace pagv_amr_core

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pagv_amr_core::AMRCoreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
