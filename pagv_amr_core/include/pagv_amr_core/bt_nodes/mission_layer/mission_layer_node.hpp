#ifndef PAGV_AMR_CORE__BT_NODES__MISSION_LAYER__MISSION_LAYER_NODE_HPP_
#define PAGV_AMR_CORE__BT_NODES__MISSION_LAYER__MISSION_LAYER_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"      
#include "behaviortree_cpp/behavior_tree.h"

namespace pagv_amr_core {

// XML: <CheckMissionActive/>
class CheckMissionActive : public BT::ConditionNode 
{
public:
    CheckMissionActive(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <ExecuteOrderNode/>
class ExecuteOrderNode : public BT::SyncActionNode 
{
public:
    ExecuteOrderNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// Registration
void RegisterMissionLayerNodes(BT::BehaviorTreeFactory& factory);

} // namespace pagv_amr_core

#endif
