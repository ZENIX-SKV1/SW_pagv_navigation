#ifndef PAGV_AMR_CORE__BT_NODES__MODE_LAYER__MODE_LAYER_NODE_HPP_
#define PAGV_AMR_CORE__BT_NODES__MODE_LAYER__MODE_LAYER_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"      
#include "behaviortree_cpp/behavior_tree.h"

namespace pagv_amr_core 
{

// ============ Emergency Mode ============
class CheckEmergency : public BT::ConditionNode 
{
public:
    CheckEmergency(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

class HandleEmergency : public BT::SyncActionNode 
{
public:
    HandleEmergency(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// ============ Manual Mode ============
class CheckManualMode : public BT::ConditionNode 
{
public:
    CheckManualMode(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

class HandleManualMode : public BT::SyncActionNode 
{
public:
    HandleManualMode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// ============ Auto Mode ============
class CheckAutoMode : public BT::ConditionNode 
{
public:
    CheckAutoMode(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Condition ID="IsAutoReady"/>
class IsAutoReady : public BT::ConditionNode 
{
public:
    IsAutoReady(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Action ID="WaitMission"/>
class WaitMission : public BT::SyncActionNode 
{
public:
    WaitMission(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Condition ID="IsAutoMoving"/>
class IsAutoMoving : public BT::ConditionNode 
{
public:
    IsAutoMoving(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <Action ID="ExecuteMission"/>
class ExecuteMission : public BT::SyncActionNode 
{
public:
    ExecuteMission(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// Registration
void RegisterModeLayerNodes(BT::BehaviorTreeFactory& factory);

} // namespace pagv_amr_core

#endif
