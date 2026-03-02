#ifndef PAGV_AMR_CORE__BT_NODES__SYSTEM_LAYER__SYSTEM_LAYER_NODE_HPP_
#define PAGV_AMR_CORE__BT_NODES__SYSTEM_LAYER__SYSTEM_LAYER_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"      
#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace pagv_amr_core 
{

// Condition: System Not Initialized
class CheckSystemNotInitialized : public BT::ConditionNode
{
public:
    CheckSystemNotInitialized(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// Action: System Initialization (Single-node sequence)
// 전체 타임아웃 60초 초과 시 FAULT_DETECTED=true, FAILURE 반환
class InitializeSystem : public BT::ActionNodeBase
{
public:
    InitializeSystem(const std::string& name, const BT::NodeConfig& config)
        : BT::ActionNodeBase(name, config)
        , _start_time(rclcpp::Time(0, 0, RCL_STEADY_TIME)) {}

    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
    void halt() override { _start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME); }

private:
    rclcpp::Time _start_time;
    static constexpr double TOTAL_TIMEOUT_SEC = 60.0;
};


// Condition: System Initialized?
// SYSTEM_INITIALIZED=true 이면 SUCCESS -> 초기화 시퀀스 건너뜀
class CheckSystemInitialized : public BT::ConditionNode
{
public:
    CheckSystemInitialized(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};



// Registration
void RegisterSystemLayerNodes(BT::BehaviorTreeFactory& factory);

} // namespace pagv_amr_core

#endif