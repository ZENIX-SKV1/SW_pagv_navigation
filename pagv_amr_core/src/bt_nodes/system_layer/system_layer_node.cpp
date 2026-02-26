#include "pagv_amr_core/bt_nodes/system_layer/system_layer_node.hpp"
#include "pagv_amr_core/blackboard/blackboard_keys.hpp"

namespace pagv_amr_core {

// ============ System Initialized Check ============
BT::NodeStatus CheckSystemInitialized::tick()
{
    auto initialized = config().blackboard->get<bool>(blackboard::SYSTEM_INITIALIZED);
    return initialized ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ============ System Initialization ============
BT::NodeStatus InitializeSystem::tick()
{
    // System initialization sequence
    // 1. VCU Power On
    // 2. VCU HW Test
    // 3. NCU Sensor Init
    // 4. Localization
    // 5. Network Connect
    
    // 초기화 완료 표시
    config().blackboard->set(blackboard::SYSTEM_INITIALIZED, true);
    config().blackboard->set(blackboard::NCU_STATUS, 10);  // AUTO_REQUEST
    
    return BT::NodeStatus::SUCCESS;
}

// ============ Registration ============
void RegisterSystemLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CheckSystemInitialized>("CheckSystemInitialized");
    factory.registerNodeType<InitializeSystem>("InitializeSystem");
}

} // namespace pagv_amr_core
