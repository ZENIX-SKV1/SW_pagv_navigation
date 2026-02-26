#include "pagv_amr_core/bt_nodes/navigation_layer/navigation_layer_node.hpp"
#include "pagv_amr_core/blackboard/blackboard_keys.hpp"

namespace pagv_amr_core {

// XML: <Condition ID="NavigationActive"/>
BT::NodeStatus NavigationActive::tick()
{
    auto ncu_status = config().blackboard->get<int>(blackboard::NCU_STATUS);
    auto mission_active = config().blackboard->get<bool>(blackboard::MISSION_ACTIVE);
    
    // Navigation은 Auto Moving 상태에서만 활성화
    bool navigation_active = (ncu_status == 100) && mission_active;
    
    return navigation_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML: <NavigationControlNode/>
BT::NodeStatus NavigationControlNode::tick()
{
    // Navigation control 로직
    // - Localization quality check
    // - Path following
    
    auto localization_quality = config().blackboard->get<double>(blackboard::LOCALIZATION_QUALITY);
    
    if (localization_quality > 0.5) {
        // Localization lost - stop
        config().blackboard->set(blackboard::TARGET_SPEED, 0.0);
        return BT::NodeStatus::FAILURE;
    } else if (localization_quality > 0.05) {
        // Bad localization - reduce speed
        config().blackboard->set(blackboard::TARGET_SPEED, 0.5);
    }
    
    auto goal_reached = config().blackboard->get<bool>(blackboard::GOAL_REACHED);
    
    if (goal_reached) {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

// XML: <PublishCmdVelNode/>
BT::NodeStatus PublishCmdVelNode::tick()
{
    // cmd_vel 발행은 AMR Core Node에서 처리
    // 여기서는 발행 가능 상태만 체크
    
    return BT::NodeStatus::SUCCESS;
}

// ============ Registration ============
void RegisterNavigationLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<NavigationActive>("NavigationActive");
    factory.registerNodeType<NavigationControlNode>("NavigationControlNode");
    factory.registerNodeType<PublishCmdVelNode>("PublishCmdVelNode");
}

} // namespace pagv_amr_core
