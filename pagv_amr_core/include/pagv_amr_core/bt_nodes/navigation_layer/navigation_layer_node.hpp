#ifndef PAGV_AMR_CORE__BT_NODES__NAVIGATION_LAYER__NAVIGATION_LAYER_NODE_HPP_
#define PAGV_AMR_CORE__BT_NODES__NAVIGATION_LAYER__NAVIGATION_LAYER_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"      
#include "behaviortree_cpp/behavior_tree.h"

namespace pagv_amr_core 
{

// XML: <Condition ID="NavigationActive"/>
class NavigationActive : public BT::ConditionNode 
{
public:
    NavigationActive(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <NavigationControlNode/>
class NavigationControlNode : public BT::SyncActionNode 
{
public:
    NavigationControlNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// XML: <PublishCmdVelNode/>
class PublishCmdVelNode : public BT::SyncActionNode 
{
public:
    PublishCmdVelNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }
};

// Registration
void RegisterNavigationLayerNodes(BT::BehaviorTreeFactory& factory);

} // namespace pagv_amr_core

#endif
