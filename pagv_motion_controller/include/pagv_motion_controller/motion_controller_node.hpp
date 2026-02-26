#ifndef PAGV_MOTION_CONTROLLER__MOTION_CONTROLLER_NODE_HPP_
#define PAGV_MOTION_CONTROLLER__MOTION_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <array>
#include <cmath>

namespace pagv_motion_controller {

struct VehicleConfig {
    double wheelbase_l1{1.65};       // 1축 → 중심 거리
    double wheelbase_l2{3.0};       // 2축 → 중심 거리
    double track_width{2.533};        // 좌우 바퀴 간격
    double max_steering_angle{0.5759};  // 33° = 0.5759 rad
};

class MotionControllerNode : public rclcpp::Node {
public:
    MotionControllerNode();
    ~MotionControllerNode() = default;

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // 8륜 4축 All-Wheel Steering (ICR 일치)
    void compute_8wheel_control(double linear_vel, double angular_vel,
                                std::array<double, 4>& steering_angles,
                                std::array<double, 2>& wheel_speeds);
    
    // Publishers - Isaac Sim으로 직접 전송
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    VehicleConfig config_;
};

} // namespace pagv_motion_controller

#endif
