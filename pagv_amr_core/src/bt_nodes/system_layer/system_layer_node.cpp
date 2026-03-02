#include "pagv_amr_core/bt_nodes/system_layer/system_layer_node.hpp"
#include "pagv_amr_core/blackboard/blackboard_keys.hpp"

namespace pagv_amr_core {

// CheckSystemNotInitialized
BT::NodeStatus CheckSystemNotInitialized::tick()
{
    auto blackboard = config().blackboard;

    bool initialized = blackboard->get<bool>(blackboard::SYSTEM_INITIALIZED);

    bool reset_requested = blackboard->get<bool>("system_reset_requested");

    if (!initialized || reset_requested) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// InitializeSystem
BT::NodeStatus InitializeSystem::tick()
{
    auto blackboard = config().blackboard;

    // Clock을 static으로 한 번만 생성
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto now = steady_clock.now();
    // 시작 시간 기록 (first tick)
    if (_start_time == rclcpp::Time(0, 0, RCL_STEADY_TIME)) {
        _start_time = now;
        RCLCPP_INFO(rclcpp::get_logger("BT"), "[SysInit] Initialization sequence started.");
    }

    double elapsed = (now - _start_time).seconds();

    // 전체 타임아웃 (60s)
    if (elapsed > TOTAL_TIMEOUT_SEC) {
        RCLCPP_ERROR(rclcpp::get_logger("BT"),
            "[SysInit] TIMEOUT after %.0fs! Triggering FAULT.", elapsed);
        blackboard->set(blackboard::FAULT_DETECTED, true);
        _start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME);
        return BT::NodeStatus::FAILURE;
    }

    // Phase1: VCU Alive
    bool vcu_alive = blackboard->get<bool>("vcu_alive");
    if (!vcu_alive) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BT"), steady_clock, 2000,
            "[SysInit] Phase1: Waiting for VCU Alive... (%.0fs)", elapsed);
        return BT::NodeStatus::RUNNING;
    }

    // VCU HW Test
    bool vcu_hw_ok = blackboard->get<bool>("vcu_hardware_status_ok");
    if (!vcu_hw_ok) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("BT"),
            "[SysInit] Phase1: Critical HW Fault reported by VCU!");
        blackboard->set(blackboard::FAULT_DETECTED, true);
        return BT::NodeStatus::FAILURE;
    }

    // Sensor Init
    bool radar_l = blackboard->get<bool>("radar_l_ready");
    bool radar_r = blackboard->get<bool>("radar_r_ready");
    bool cam_f   = blackboard->get<bool>("cam_f_ready");
    bool cam_b   = blackboard->get<bool>("cam_b_ready");
    bool cam_l   = blackboard->get<bool>("cam_l_ready");
    bool cam_r   = blackboard->get<bool>("cam_r_ready");
    bool lidar   = blackboard->get<bool>("lidar_ready");
    bool gnss    = blackboard->get<bool>("gnss_ready");

    bool sensors_ready = radar_l && radar_r && cam_f && cam_b && cam_l && cam_r && lidar && gnss;
    if (!sensors_ready) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BT"), steady_clock, 2000,
            "[SysInit] Phase2: Sensor init... (%.0fs) "
            "Radar[L:%d R:%d] Cam[F:%d B:%d L:%d R:%d] LiDAR:%d GNSS:%d",
            elapsed, radar_l, radar_r, cam_f, cam_b, cam_l, cam_r, lidar, gnss);
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO_ONCE(rclcpp::get_logger("BT"), "[SysInit] Phase2: All sensors ready.");

    // Localization Health
    double pose_health = blackboard->get<double>("pose_health");
    if (pose_health < 85.0) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BT"), steady_clock, 2000,
            "[SysInit] Phase3: Localization health %.1f%% < 85%%. Waiting for RTK Fix...",
            pose_health);
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO_ONCE(rclcpp::get_logger("BT"),
        "[SysInit] Phase3: Localization OK (health=%.1f%%).", pose_health);

    // FMS Network Connect
    bool fms_connected = blackboard->get<bool>("fms_connected");
    if (!fms_connected) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BT"), steady_clock, 5000,
            "[SysInit] Phase4: Connecting to FMS via WiFi/MQTT... (%.0fs)", elapsed);
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO_ONCE(rclcpp::get_logger("BT"), "[SysInit] Phase4: FMS connected.");

    // FMS Auto Ready Permission
    blackboard->set(blackboard::NCU_STATUS, 5);  // AUTO_STOP

    bool auto_ready_permitted = blackboard->get<bool>("fms_auto_ready_permitted");
    if (!auto_ready_permitted) {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BT"), steady_clock, 5000,
            "[SysInit] Phase5: Waiting for FMS 'Auto Ready' permission... (%.0fs)", elapsed);
        return BT::NodeStatus::RUNNING;
    }

    // Initialization Complete
    RCLCPP_INFO(rclcpp::get_logger("BT"),
        "[SysInit] *** Initialization Complete (%.0fs) *** "
        "MISSION_ACTIVE=true, NCU_STATUS=AUTO_REQUEST", elapsed);

    blackboard->set(blackboard::SYSTEM_INITIALIZED, true);
    blackboard->set("system_reset_requested",       false);
    blackboard->set(blackboard::NCU_STATUS,         10);  // AUTO_REQUEST
    blackboard->set(blackboard::MISSION_ACTIVE,     true);
    blackboard->set(blackboard::FAULT_DETECTED,     false);

    _start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    return BT::NodeStatus::SUCCESS;
}

// CheckSystemInitialized
BT::NodeStatus CheckSystemInitialized::tick()
{
    bool initialized = config().blackboard->get<bool>(blackboard::SYSTEM_INITIALIZED);
    return initialized ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// Registration
void RegisterSystemLayerNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CheckSystemNotInitialized>("CheckSystemNotInitialized");
    factory.registerNodeType<InitializeSystem>("InitializeSystem");
    factory.registerNodeType<CheckSystemInitialized>("CheckSystemInitialized");
}

} // namespace pagv_amr_core