#include "pagv_amr_core/navigation/navigation_manager.hpp"
#include <algorithm>
#include <iostream>

namespace pagv_amr_core {

NavigationManager::NavigationManager()
    : config_()
{
    std::cout << "[NavigationManager] Initialized with Pure Pursuit" << std::endl;
}

NavigationManager::NavigationManager(const NavigationConfig& config)
    : config_(config)
{
}

void NavigationManager::set_goal(const NodePosition& goal)
{
    goal_pose_.x = goal.x;
    goal_pose_.y = goal.y;
    goal_pose_.theta = goal.theta;
    has_goal_ = true;
    use_arc_ = false;  
    goal_reached_ = false;    
    
    std::cout << "[NavigationManager] New goal: (" 
              << goal.x << ", " << goal.y << ", " << goal.theta << ")" << std::endl;
}

void NavigationManager::set_arc_goal(double start_x, double start_y, double start_theta,
                                     double target_x, double target_y,
                                     double center_x, double center_y)
{
    target_x_ = target_x;
    target_y_ = target_y;
    
    // compute arc parameter
    arc_center_x_ = center_x;
    arc_center_y_ = center_y;
    arc_radius_ = std::hypot(target_x - center_x, target_y - center_y);
    arc_start_angle_ = std::atan2(start_y - center_y, start_x - center_x);
    arc_end_angle_ = std::atan2(target_y - center_y, target_x - center_x);
    
    // rotation direction
    double dx = center_x - start_x;
    double dy = center_y - start_y;
    
    // 로봇헤딩 기준좌표계로 중심점변환
    double local_center_y = -std::sin(start_theta) * dx + std::cos(start_theta) * dy;
    
    // 중심점이 오른쪽에 있으면 좌회전(CCW), 왼쪽에 있으면 우회전(CW)
    arc_clockwise_ = (local_center_y < 0);
    
    use_arc_ = true;
    has_goal_ = true;
    goal_reached_ = false;
    
    std::cout << "[NavigationManager] Arc goal set" << std::endl;
    std::cout << "  Center: (" << center_x << ", " << center_y << ")" << std::endl;
    std::cout << "  Radius: " << arc_radius_ << std::endl;
    std::cout << "  Direction: " << (arc_clockwise_ ? "CW" : "CCW") << std::endl;
}

void NavigationManager::set_current_pose(const Pose& pose)
{
    current_pose_ = pose;
}

geometry_msgs::msg::Twist NavigationManager::compute_velocity_command()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    if (!has_goal_) 
    {
        return cmd_vel;
    }
    
    if (goal_reached_) 
    {
        return cmd_vel;
    }
    
    if (use_arc_) 
    {
        // 곡선 경로 제어
        compute_arc_velocity(cmd_vel.linear.x, cmd_vel.angular.z);
    } 
    else 
    {
        // 직선 경로 제어
        compute_straight_velocity(cmd_vel.linear.x, cmd_vel.angular.z);
    }
    
    return cmd_vel;
}

void NavigationManager::compute_straight_velocity(double& linear, double& angular)
{
    double dx = goal_pose_.x - current_pose_.x;
    double dy = goal_pose_.y - current_pose_.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Lookahead point
    double lookahead = config_.lookahead_distance;
    double lx = current_pose_.x + lookahead * std::cos(current_pose_.theta);
    double ly = current_pose_.y + lookahead * std::sin(current_pose_.theta);
    
    // 목표가 lookahead보다 가까우면 목표를 lookahead point로 사용
    if (distance < lookahead) 
    {
        lx = goal_pose_.x;
        ly = goal_pose_.y;
        lookahead = distance;
    }
    
    // Pure Pursuit Curvature 계산
    double curvature = calculate_pure_pursuit(
        current_pose_.x, current_pose_.y, current_pose_.theta,
        lx, ly, lookahead);
    
    // Linear velocity (거리 기반 감속)
    double linear_vel = compute_linear_velocity();
    
    // Angular velocity
    double angular_vel = linear_vel * curvature;
    
    // Limit angular velocity
    const double max_angular_vel = 1.0;
    angular_vel = std::clamp(angular_vel, -max_angular_vel, max_angular_vel);
    
    linear = linear_vel;
    angular = angular_vel;
}

void NavigationManager::compute_arc_velocity(double& linear, double& angular)
{
    constexpr double position_threshold = 0.05;  // 5cm
    
    // 목표도달 확인
    double dist_to_target = std::hypot(target_x_ - current_pose_.x, target_y_ - current_pose_.y);
    if (dist_to_target < position_threshold) 
    {
        linear = 0.0;
        angular = 0.0;
        goal_reached_ = true;
        std::cout << "[NavigationManager] Arc goal reached!" << std::endl;
        return;
    }
    
    // 반경에 비례한 동적 룩어헤드 거리
    double look_ahead_dist = std::clamp(arc_radius_ * 0.3, 0.5, 2.0);
    
    // 반경에 따른 속도 조정
    double arc_follow_speed = 2.0;
    if (arc_radius_ < 10.0) 
    {
        arc_follow_speed = std::max(1.0, arc_radius_ * 0.2);
    }
    
    // 현재 로봇 위치에서 원 궤적까지의 오차 계산
    double dx_to_center = current_pose_.x - arc_center_x_;
    double dy_to_center = current_pose_.y - arc_center_y_;
    double dist_to_center = std::sqrt(dx_to_center * dx_to_center + dy_to_center * dy_to_center);
    double radius_error = arc_radius_ - dist_to_center;
    
    // 반경 오차가 크면 직접 복귀 제어
    if (std::abs(radius_error) > arc_radius_ * 0.3) 
    {
        std::cout << "[NavigationManager] Large radius error: " << radius_error
                 << "m. Applying correction mode." << std::endl;
        
        double angle_to_center = std::atan2(dy_to_center, dx_to_center);
        double target_angle = angle_to_center + (radius_error > 0 ? M_PI : 0);
        
        double angle_diff = target_angle - current_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        linear = std::min(1.0, std::abs(radius_error));
        angular = std::clamp(angle_diff * 2.0, -1.0, 1.0);
        return;
    }
    
    // Pure Pursuit 계산
    double current_angle = std::atan2(current_pose_.y - arc_center_y_, current_pose_.x - arc_center_x_);
    double angle_step = look_ahead_dist / arc_radius_;
    
    double target_angle = arc_clockwise_ ? (current_angle - angle_step) : (current_angle + angle_step);
    
    // target_angle 불연속 구간 보정
    double angle_to_target = target_angle - current_angle;
    while (angle_to_target > M_PI) angle_to_target -= 2.0 * M_PI;
    while (angle_to_target < -M_PI) angle_to_target += 2.0 * M_PI;
    target_angle = current_angle + angle_to_target;
    
    double lx = arc_center_x_ + arc_radius_ * std::cos(target_angle);
    double ly = arc_center_y_ + arc_radius_ * std::sin(target_angle);
    
    double curvature = calculate_pure_pursuit(current_pose_.x, current_pose_.y, current_pose_.theta, 
                                              lx, ly, look_ahead_dist);
    
    // 곡률 제한
    double max_curvature = 1.0 / arc_radius_ * 1.5;
    if (std::abs(curvature) > max_curvature) {
        curvature = std::copysign(max_curvature, curvature);
    }
    
    linear = arc_follow_speed;
    angular = linear * curvature;
    
    // 각속도 최종 제한
    double max_angular = std::min(1.5, arc_follow_speed / arc_radius_ * 2.0);
    angular = std::clamp(angular, -max_angular, max_angular);
}


bool NavigationManager::is_goal_reached() const
{
    if (!has_goal_) 
    {
        return false;
    }
    
    if (use_arc_) 
    {
        // 곡선경로는 compute_arc_velocity에서 goal_reached_ 설정
        return goal_reached_;
    } 
    else 
    {
        // 직선 경로
        double distance = get_distance_to_goal();
        double angle_diff = std::abs(normalize_angle(goal_pose_.theta - current_pose_.theta));
        
        bool position_ok = distance < config_.position_tolerance;
        bool orientation_ok = angle_diff < config_.orientation_tolerance;
        
        return position_ok && orientation_ok;
    }
}

double NavigationManager::get_distance_to_goal() const
{
    if (use_arc_) 
    {
        return std::hypot(target_x_ - current_pose_.x, target_y_ - current_pose_.y);
    } 
    else 
    {
        return std::hypot(goal_pose_.x - current_pose_.x, goal_pose_.y - current_pose_.y);
    }
}

double NavigationManager::normalize_angle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double NavigationManager::compute_linear_velocity() const
{
    double distance = get_distance_to_goal();
    
    const double decel_distance = 2.0;
    
    if (distance > decel_distance) 
    {
        return config_.max_speed;
    } 
    else 
    {
        double speed = config_.min_speed + (config_.max_speed - config_.min_speed) * (distance / decel_distance);
        return std::max(speed, config_.min_speed);
    }
}

// Pure Pursuit 계산(에뮬레이터 navigation.cpp 로직)
double NavigationManager::calculate_pure_pursuit(double cx, double cy, double ct,
                                                  double lx, double ly, double ld)
{
    double dx = lx - cx;
    double dy = ly - cy;
    
    // 로봇 로컬 좌표계 변환
    // double local_x = std::cos(ct) * dx + std::sin(ct) * dy;
    double local_y = -std::sin(ct) * dx + std::cos(ct) * dy;
    
    // local_x가 음수더라도 곡률 계산 수행
    if (ld < 0.1) ld = 0.1;
    
    // Pure Pursuit 곡률
    double curvature = (2.0 * local_y) / (ld * ld);
    
    return curvature;
}

// 반경 오차 보정(에뮬레이터 navigation.cpp 로직)
void NavigationManager::apply_radius_error_correction(double radius_error,
                                                       double& linear_speed,
                                                       double& angular_speed)
{
    const double max_radius_error = 1.5;
    const double max_speed_reduction = 0.5;
    const double max_angular_increase = 0.5;
    
    double error_ratio = std::min(std::abs(radius_error) / max_radius_error, 1.0);
    
    // 선속도 보정
    linear_speed *= (1.0 - max_speed_reduction * error_ratio);
    
    // 각속도 보정
    double correction_factor = 1.0 + (max_angular_increase - 1.0) * error_ratio;
    angular_speed *= correction_factor;
}

} 
