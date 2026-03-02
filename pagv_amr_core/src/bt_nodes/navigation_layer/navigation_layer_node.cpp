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
BT::NodeStatus NavigationControlNode::onStart()
{
    return onRunning();
}

BT::NodeStatus NavigationControlNode::onRunning()
{
    auto bb = config().blackboard;

    // localization_quality: 0~100 스케일 (pose_health와 동일 기준)
    // 85 이상 = RTK 정상, 50~85 = 감속, 50 미만 = 정지
    auto localization_quality = bb->get<double>(blackboard::LOCALIZATION_QUALITY);

    if (localization_quality < 50.0) {
        // Localization lost → 정지
        bb->set(blackboard::TARGET_SPEED, 0.0);
        RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger("BT"),
            "[Nav] Localization lost (" << localization_quality << ") - stopping");
        return BT::NodeStatus::FAILURE;
    } else if (localization_quality < 85.0) {
        // 측위 불안정 → 감속
        bb->set(blackboard::TARGET_SPEED, 0.5);
    }

    auto goal_reached = bb->get<bool>(blackboard::GOAL_REACHED);
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

//Registration
void RegisterNavigationLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<NavigationActive>("NavigationActive");
    factory.registerNodeType<NavigationControlNode>("NavigationControlNode");
    factory.registerNodeType<PublishCmdVelNode>("PublishCmdVelNode");
}

} 
