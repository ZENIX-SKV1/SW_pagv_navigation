#ifndef PAGV_AMR_CORE__BT_NODES__SYSTEM_LAYER__SYSTEM_LAYER_NODE_HPP_
#define PAGV_AMR_CORE__BT_NODES__SYSTEM_LAYER__SYSTEM_LAYER_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"      
#include "behaviortree_cpp/behavior_tree.h"

namespace pagv_amr_core 
{

// ============ System Initialized Check ============
class CheckSystemInitialized : public BT::ConditionNode 
{
public:
    CheckSystemInitialized(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// ============ System Initialization ============
class InitializeSystem : public BT::SyncActionNode 
{
public:
    InitializeSystem(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// Registration
void RegisterSystemLayerNodes(BT::BehaviorTreeFactory& factory);

} // namespace pagv_amr_core

#endif
