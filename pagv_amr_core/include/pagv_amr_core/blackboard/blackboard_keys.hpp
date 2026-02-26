#ifndef PAGV_AMR_CORE__BLACKBOARD__BLACKBOARD_KEYS_HPP_
#define PAGV_AMR_CORE__BLACKBOARD__BLACKBOARD_KEYS_HPP_

#include <string>

namespace pagv_amr_core {
namespace blackboard {

// Mode Layer
constexpr const char* MODE = "mode";
constexpr const char* NCU_STATUS = "ncu_status";
constexpr const char* EMERGENCY_ACTIVE = "emergency_active";

// Safety Layer
constexpr const char* FAULT_DETECTED = "fault_detected";
constexpr const char* COLLISION_RISK = "collision_risk";
constexpr const char* BATTERY_SOC = "battery_soc";
constexpr const char* TARGET_SPEED = "target_speed";

// System Layer
constexpr const char* SYSTEM_INITIALIZED = "system_initialized";

// Mission Layer
constexpr const char* MISSION_ACTIVE = "mission_active";
constexpr const char* MISSION_COMPLETE = "mission_complete";

// Navigation Layer
constexpr const char* GOAL_REACHED = "goal_reached";
constexpr const char* LOCALIZATION_QUALITY = "localization_quality";
constexpr const char* CURRENT_X = "current_x";
constexpr const char* CURRENT_Y = "current_y";
constexpr const char* CURRENT_THETA = "current_theta";

} 
} 

#endif
