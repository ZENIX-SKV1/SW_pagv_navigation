#ifndef PAGV_LOCALIZER__LOCALIZER_NODE_HPP_
#define PAGV_LOCALIZER__LOCALIZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>

namespace pagv_localizer {

class LocalizerNode : public rclcpp::Node {
public:
    LocalizerNode();
    ~LocalizerNode() = default;

private:
    void encoder_steering_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void encoder_wheel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void dead_reckoning_update();
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr encoder_steering_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr encoder_wheel_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr dr_timer_;
    
    // Dead Reckoning state
    std::vector<float> encoder_steering_;
    std::vector<float> encoder_wheel_;
    std::vector<float> prev_encoder_wheel_{0.0f, 0.0f};
    
    double dr_x_{0.0};
    double dr_y_{0.0};
    double dr_theta_{0.0};
    
    bool encoder_steering_updated_{false};
    bool encoder_wheel_updated_{false};
    
    rclcpp::Time prev_time_;
    
    // Configuration
    double wheel_radius_{0.25};
    double encoder_resolution_{1000.0};
    double wheelbase_{6.5};
};

} // namespace pagv_localizer

#endif
