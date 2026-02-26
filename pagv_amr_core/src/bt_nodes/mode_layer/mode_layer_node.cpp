#include "pagv_amr_core/bt_nodes/mode_layer/mode_layer_node.hpp"
#include "pagv_amr_core/blackboard/blackboard_keys.hpp"

namespace pagv_amr_core {

// ============ Emergency Mode ============
BT::NodeStatus CheckEmergency::tick()
{
    auto emergency = config().blackboard->get<bool>(blackboard::EMERGENCY_ACTIVE);
    return emergency ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus HandleEmergency::tick()
{
    // Emergency Stop 처리
    config().blackboard->set(blackboard::NCU_STATUS, 200);  
    config().blackboard->set(blackboard::TARGET_SPEED, 0.0);
    return BT::NodeStatus::SUCCESS;
}

// ============ Manual Mode ============
BT::NodeStatus CheckManualMode::tick()
{
    auto mode = config().blackboard->get<std::string>(blackboard::MODE);
    return (mode == "MANUAL") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus HandleManualMode::tick()
{
    // Manual Mode 처리 (Joystick Control)
    config().blackboard->set(blackboard::NCU_STATUS, 150);
    return BT::NodeStatus::RUNNING;
}

// ============ Auto Mode ============
BT::NodeStatus CheckAutoMode::tick()
{
    auto mode = config().blackboard->get<std::string>(blackboard::MODE);
    return (mode == "AUTO") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <Condition ID="IsAutoReady"/>
BT::NodeStatus IsAutoReady::tick()
{
    auto ncu_status = config().blackboard->get<int>(blackboard::NCU_STATUS);
    return (ncu_status == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <Action ID="WaitMission"/>
BT::NodeStatus WaitMission::tick()
{
    // Mission 대기 중
    return BT::NodeStatus::RUNNING;
}

// XML: <Condition ID="IsAutoMoving"/>
BT::NodeStatus IsAutoMoving::tick()
{
    auto ncu_status = config().blackboard->get<int>(blackboard::NCU_STATUS);
    return (ncu_status == 100) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <Action ID="ExecuteMission"/>
BT::NodeStatus ExecuteMission::tick()
{
    // Mission execution 로직
    auto mission_complete = config().blackboard->get<bool>(blackboard::MISSION_COMPLETE);
    
    if (mission_complete) {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

// ============ Registration ============
void RegisterModeLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CheckEmergency>("CheckEmergency");
    factory.registerNodeType<HandleEmergency>("HandleEmergency");
    factory.registerNodeType<CheckManualMode>("CheckManualMode");
    factory.registerNodeType<HandleManualMode>("HandleManualMode");
    factory.registerNodeType<CheckAutoMode>("CheckAutoMode");
    factory.registerNodeType<IsAutoReady>("IsAutoReady");
    factory.registerNodeType<WaitMission>("WaitMission");
    factory.registerNodeType<IsAutoMoving>("IsAutoMoving");
    factory.registerNodeType<ExecuteMission>("ExecuteMission");
}

} // namespace pagv_amr_core
