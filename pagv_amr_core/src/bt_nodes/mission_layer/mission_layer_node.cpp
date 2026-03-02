#include "pagv_amr_core/bt_nodes/mission_layer/mission_layer_node.hpp"
#include "pagv_amr_core/blackboard/blackboard_keys.hpp"

namespace pagv_amr_core {

//Mission Active Check
BT::NodeStatus CheckMissionActive::tick()
{
    auto mission_active = config().blackboard->get<bool>(blackboard::MISSION_ACTIVE);
    return mission_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

//Execute Order Node
BT::NodeStatus ExecuteOrderNode::onStart()
{
    auto mission_complete = config().blackboard->get<bool>(blackboard::MISSION_COMPLETE);
    return mission_complete ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteOrderNode::onRunning()
{
    auto mission_complete = config().blackboard->get<bool>(blackboard::MISSION_COMPLETE);
    return mission_complete ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

//Registration
void RegisterMissionLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CheckMissionActive>("CheckMissionActive");
    factory.registerNodeType<ExecuteOrderNode>("ExecuteOrderNode");
}

}