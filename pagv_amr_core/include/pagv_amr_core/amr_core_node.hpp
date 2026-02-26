#ifndef PAGV_AMR_CORE__AMR_CORE_NODE_HPP_
#define PAGV_AMR_CORE__AMR_CORE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pagv_amr_core/vda5050_protocol.hpp"
#include "pagv_amr_core/node_edge_info.h"
#include "pagv_amr_core/iamr.hpp"

#include "pagv_amr_core/order_execution/order_execution_manager.hpp"
#include "pagv_amr_core/navigation/navigation_manager.hpp"

// BT Layers
#include "pagv_amr_core/bt_nodes/mode_layer/mode_layer_node.hpp"
#include "pagv_amr_core/bt_nodes/safety_layer/safety_layer_node.hpp"
#include "pagv_amr_core/bt_nodes/system_layer/system_layer_node.hpp"
#include "pagv_amr_core/bt_nodes/mission_layer/mission_layer_node.hpp"
#include "pagv_amr_core/bt_nodes/navigation_layer/navigation_layer_node.hpp"

namespace pagv_amr_core 
{

class AMRCoreNode : public rclcpp::Node, public IAmr 
{
public:
    AMRCoreNode();
    ~AMRCoreNode();

    // IAmr Interface 구현
    std::string getId() const override { return agv_id_; }
    
    void setOrder(const std::vector<NodeInfo>& nodes, 
                 const std::vector<EdgeInfo>& edges, 
                 const std::vector<NodeInfo>& all_nodes) override;  
    
    void updateOrder(const std::vector<NodeInfo>& nodes, 
                    const std::vector<EdgeInfo>& edges,
                    const std::vector<NodeInfo>& all_nodes) override;  
    
    std::string getState() const override;
    std::vector<NodeInfo> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    
    void step(double dt) override;
    void updateBattery(double dt, bool is_charging) override;
    double getBatteryPercent() const override { return battery_soc_; }
    double getBatteryVoltage() const override { return battery_voltage_; }
    
    std::vector<NodeInfo> getCurrentNodes() const override;
    std::vector<NodeInfo> getCompletedNodes() const override { return completed_nodes_; }
    std::vector<EdgeInfo> getCurrentEdges() const override;
    std::vector<EdgeInfo> getCompletedEdges() const override { return completed_edges_; }
    std::string getLastNodeId() const override;
    int getLastNodeSequenceId() const override;
    
    void markNodeAsCompleted(const NodeInfo& node) override;
    void cancelOrder() override;
    
    void executeAction(const ActionInfo& action) override;
    
    // VCU 대체함수
    IVcu* getVcu() override { return nullptr; }
    
    void getEstimatedPose(double& x, double& y, double& theta) const override {
        x = current_pose_.x;
        y = current_pose_.y;
        theta = current_pose_.theta;
    }
    
    double getLinearVelocity() const override { return current_pose_.linear_velocity; }
    double getAngularVelocity() const override { return current_pose_.angular_velocity; }
    
    bool isPaused() const override { return is_paused_; }
    void pause() override { is_paused_ = true; }
    void resume() override { is_paused_ = false; }
    bool isDriving() const override;
    void setMaxSpeed(double speed) override { max_speed_override_ = speed; }
    std::string getOperatingMode() const override { return operating_mode_; }
    
    bool needsImmediateStatePublish() const override { 
        return needs_immediate_state_publish_; 
    }
    void resetImmediateStatePublishFlag() override { 
        needs_immediate_state_publish_ = false; 
    }

private:
    // ROS2 Callbacks
    void control_loop();
    void vda5050_state_publish_loop();
    void vda5050_vis_publish_loop();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // VDA5050 Protocol
    std::unique_ptr<Vda5050Protocol> vda5050_protocol_;
    
    // BehaviorTree
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    
    void init_behavior_tree();
    void register_bt_nodes();
    void update_blackboard();
    
    // Order & Navigation
    std::shared_ptr<OrderExecutionManager> order_manager_;
    std::shared_ptr<NavigationManager> nav_manager_;
    
    std::vector<NodeInfo> nodes_;
    std::vector<EdgeInfo> edges_;
    std::vector<NodeInfo> all_nodes_;
    std::vector<NodeInfo> completed_nodes_;
    std::vector<EdgeInfo> completed_edges_;
    
    std::size_t cur_idx_{0};
    std::size_t cur_edge_idx_{0};
    double wheel_base_{0.0};
    bool is_angle_adjusting_{false};
    
    void setNavigationTargetFromEdge(const EdgeInfo& edge,
        const std::vector<NodeInfo>& nodes,
        const std::vector<NodeInfo>& all_nodes);
    
    // ROS2 Communication
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr amr_status_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr vda5050_state_timer_;
    rclcpp::TimerBase::SharedPtr vda5050_vis_timer_;
    
    // State Variables
    Pose current_pose_;
    bool pose_initialized_{false};
    
    // Configuration
    std::string agv_id_;
    std::string mqtt_broker_;
    double control_rate_{20.0};
    double state_publish_rate_{1.0};
    double vis_publish_rate_{20.0};
    
    // AMR State
    bool is_paused_{false};
    double max_speed_override_{0.0};
    std::string operating_mode_{"AUTOMATIC"};
    bool needs_immediate_state_publish_{false};
    
    double battery_soc_{100.0};
    double battery_voltage_{48.0};
    
    std::string mode_{"AUTO"};
    int ncu_status_{0};
    bool emergency_active_{false};
    bool fault_detected_{false};
    bool collision_risk_{false};
    bool system_initialized_{false};
    bool mission_active_{false};
    bool mission_complete_{false};
    bool goal_reached_{false};
    double localization_quality_{0.02};
    
    // Action 실행 상태
    struct ExecutingAction 
    {
        ActionInfo info;
        std::string status;
        double start_time;
        double duration;
    };
    std::vector<ExecutingAction> executing_actions_;

    template<typename T>
    void syncFromBlackboard(const std::string& key, T& member_var, const std::string& label) 
    {
        if (!blackboard_) return;
        try 
        {
            if (blackboard_->getAny(key)) 
            {
                T bt_value = blackboard_->get<T>(key);
                if (bt_value != member_var) 
                {
                    RCLCPP_INFO(this->get_logger(), "[AMRCore] BT changed %s: %s -> %s",
                               label.c_str(), std::to_string(member_var).c_str(), std::to_string(bt_value).c_str());

                    member_var = bt_value;
                }
            }
        }
        catch (const std::exception& e) 
        {
            RCLCPP_DEBUG(this->get_logger(), "[AMRCore] Sync failed for %s: %s", key.c_str(), e.what());
        }
    }
};

}

#endif
