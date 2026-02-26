#ifndef PAGV_AMR_CORE__BT_NODES__SAFETY_LAYER__SAFETY_LAYER_NODE_HPP_
#define PAGV_AMR_CORE__BT_NODES__SAFETY_LAYER__SAFETY_LAYER_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"      
#include "behaviortree_cpp/behavior_tree.h"

namespace pagv_amr_core 
{

// ============ Fault Detection ============
class CheckFaults : public BT::ConditionNode 
{
public:
    CheckFaults(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Action ID="TriggerEmergency"/>
class TriggerEmergency : public BT::SyncActionNode 
{
public:
    TriggerEmergency(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// ============ Collision Prevention ============
class CheckCollisionRisk : public BT::ConditionNode 
{
public:
    CheckCollisionRisk(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Action ID="SlowDown"/>
class SlowDown : public BT::SyncActionNode 
{
public:
    SlowDown(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// ============ Battery Management ============
class CheckBatteryLevel : public BT::ConditionNode 
{
public:
    CheckBatteryLevel(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Action ID="RequestCharging"/>
class RequestCharging : public BT::SyncActionNode 
{
public:
    RequestCharging(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

void RegisterSafetyLayerNodes(BT::BehaviorTreeFactory& factory);

} // namespace pagv_amr_core

#endif
