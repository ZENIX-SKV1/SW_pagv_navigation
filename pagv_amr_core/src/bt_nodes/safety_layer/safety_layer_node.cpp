#include "pagv_amr_core/bt_nodes/safety_layer/safety_layer_node.hpp"
#include "pagv_amr_core/blackboard/blackboard_keys.hpp"

namespace pagv_amr_core {

// ============ Fault Detection ============
BT::NodeStatus CheckFaults::tick()
{
    auto fault = config().blackboard->get<bool>(blackboard::FAULT_DETECTED);
    return fault ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <Action ID="TriggerEmergency"/>
BT::NodeStatus TriggerEmergency::tick()
{
    // Trigger emergency stop
    config().blackboard->set(blackboard::EMERGENCY_ACTIVE, true);
    config().blackboard->set(blackboard::TARGET_SPEED, 0.0);
    return BT::NodeStatus::SUCCESS;
}

// ============ Collision Prevention ============
BT::NodeStatus CheckCollisionRisk::tick()
{
    auto collision = config().blackboard->get<bool>(blackboard::COLLISION_RISK);
    return collision ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <Action ID="SlowDown"/>
BT::NodeStatus SlowDown::tick()
{
    // Reduce speed for collision avoidance
    auto current_speed = config().blackboard->get<double>(blackboard::TARGET_SPEED);
    config().blackboard->set(blackboard::TARGET_SPEED, current_speed * 0.5);
    return BT::NodeStatus::SUCCESS;
}

// ============ Battery Management ============
BT::NodeStatus CheckBatteryLevel::tick()
{
    auto soc = config().blackboard->get<double>(blackboard::BATTERY_SOC);
    return (soc < 30.0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <Action ID="RequestCharging"/>
BT::NodeStatus RequestCharging::tick()
{
    // Request charging action
    // TODO: Send charging request to FMS
    return BT::NodeStatus::SUCCESS;
}

// ============ Registration ============
void RegisterSafetyLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CheckFaults>("CheckFaults");
    factory.registerNodeType<TriggerEmergency>("TriggerEmergency");
    factory.registerNodeType<CheckCollisionRisk>("CheckCollisionRisk");
    factory.registerNodeType<SlowDown>("SlowDown");
    factory.registerNodeType<CheckBatteryLevel>("CheckBatteryLevel");
    factory.registerNodeType<RequestCharging>("RequestCharging");
}

} // namespace pagv_amr_core
