#ifndef PAGV_AMR_CORE__BLACKBOARD__BLACKBOARD_KEYS_HPP_
#define PAGV_AMR_CORE__BLACKBOARD__BLACKBOARD_KEYS_HPP_

#include <string>

namespace pagv_amr_core {
namespace blackboard {

// Mode Layer 
constexpr const char* MODE             = "mode";
constexpr const char* NCU_STATUS       = "ncu_status";  
constexpr const char* EMERGENCY_ACTIVE = "emergency_active";

// Safety Layer 
constexpr const char* FAULT_DETECTED   = "fault_detected";
constexpr const char* COLLISION_RISK   = "collision_risk";
constexpr const char* BATTERY_SOC      = "battery_soc";
constexpr const char* TARGET_SPEED     = "target_speed";

// System Layer 
constexpr const char* SYSTEM_INITIALIZED = "system_initialized";
constexpr const char* SYSTEM_RESET_REQUESTED = "system_reset_requested";

// sensor status
constexpr const char* VCU_ALIVE               = "vcu_alive";
constexpr const char* VCU_HARDWARE_STATUS_OK  = "vcu_hardware_status_ok";
constexpr const char* RADAR_L_READY           = "radar_l_ready";
constexpr const char* RADAR_R_READY           = "radar_r_ready";
constexpr const char* CAM_F_READY             = "cam_f_ready";
constexpr const char* CAM_B_READY             = "cam_b_ready";
constexpr const char* CAM_L_READY             = "cam_l_ready";
constexpr const char* CAM_R_READY             = "cam_r_ready";
constexpr const char* GNSS_READY              = "gnss_ready";
constexpr const char* LIDAR_READY             = "lidar_ready";
constexpr const char* POSE_HEALTH             = "pose_health";
constexpr const char* FMS_CONNECTED           = "fms_connected";
constexpr const char* FMS_AUTO_READY_PERMITTED = "fms_auto_ready_permitted";

//  Mission Layer 
constexpr const char* MISSION_ACTIVE   = "mission_active";
constexpr const char* MISSION_COMPLETE = "mission_complete";

//  Navigation Layer 
constexpr const char* GOAL_REACHED         = "goal_reached";
constexpr const char* LOCALIZATION_QUALITY = "localization_quality";
constexpr const char* CURRENT_X            = "current_x";
constexpr const char* CURRENT_Y            = "current_y";
constexpr const char* CURRENT_THETA        = "current_theta";
} // namespace blackboard
} // namespace pagv_amr_core

#endif // PAGV_AMR_CORE__BLACKBOARD__BLACKBOARD_KEYS_HPP_
