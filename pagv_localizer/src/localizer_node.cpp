#include "pagv_localizer/localizer_node.hpp"

namespace pagv_localizer 
{

LocalizerNode::LocalizerNode()
    : Node("localizer_node")
{
    RCLCPP_INFO(this->get_logger(), "[Localizer] Initializing Dead Reckoning...");
    
    // Parameters
    this->declare_parameter("wheel_radius", 0.25);
    this->declare_parameter("encoder_resolution", 1000.0);
    this->declare_parameter("wheelbase", 6.5);
    
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    encoder_resolution_ = this->get_parameter("encoder_resolution").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    
    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    // Subscribers - Isaac Sim 엔코더 피드백
    encoder_steering_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/isaac/encoder/steering", 10,
        std::bind(&LocalizerNode::encoder_steering_callback, this, std::placeholders::_1));
    
    encoder_wheel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/isaac/encoder/wheel", 10,
        std::bind(&LocalizerNode::encoder_wheel_callback, this, std::placeholders::_1));
    
    // Dead Reckoning timer (50Hz)
    dr_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LocalizerNode::dead_reckoning_update, this));
    
    prev_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "[Localizer] Ready");
    RCLCPP_INFO(this->get_logger(), "  - Wheel radius: %.3f m", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "  - Encoder resolution: %.0f pulses/rev", encoder_resolution_);
    RCLCPP_INFO(this->get_logger(), "  - Wheelbase: %.2f m", wheelbase_);
}

void LocalizerNode::encoder_steering_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 8) {
        RCLCPP_WARN(this->get_logger(), "Invalid steering encoder size: %zu (expected 8)", msg->data.size());
        return;
    }
    
    encoder_steering_ = msg->data;
    encoder_steering_updated_ = true;
}

void LocalizerNode::encoder_wheel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 2) {
        RCLCPP_WARN(this->get_logger(), "Invalid wheel encoder size: %zu (expected 2)", msg->data.size());
        return;
    }
    
    encoder_wheel_ = msg->data;
    encoder_wheel_updated_ = true;
}

void LocalizerNode::dead_reckoning_update()
{
    // 엔코더 데이터 확인
    if (!encoder_steering_updated_ || !encoder_wheel_updated_) {
        return;
    }
    
    auto current_time = this->now();
    double dt = (current_time - prev_time_).seconds();
    
    if (dt <= 0.0) {
        prev_time_ = current_time;
        return;
    }
    
    //휠 엔코더->속도 계산
    double delta_pulse_2 = encoder_wheel_[0] - prev_encoder_wheel_[0];
    double delta_pulse_3 = encoder_wheel_[1] - prev_encoder_wheel_[1];
    
    //pulse->회전수->거리->속도
    double wheel_velocity_2 = (delta_pulse_2 / encoder_resolution_) * 
                               (2.0 * M_PI * wheel_radius_) / dt;
    double wheel_velocity_3 = (delta_pulse_3 / encoder_resolution_) * 
                               (2.0 * M_PI * wheel_radius_) / dt;
    
    double avg_velocity = (wheel_velocity_2 + wheel_velocity_3) / 2.0;
    
    //조향각->각속도 계산
    // 1열(좌/우):0,1 4열(좌/우):6,7
    double delta_f = (encoder_steering_[0] + encoder_steering_[1]) / 2.0; 
    double delta_r = (encoder_steering_[6] + encoder_steering_[7]) / 2.0;
    
    //각속도 계산
    double angular_velocity = 0.0;
    if (std::abs(avg_velocity) > 0.01) 
    {
        // 전륜과 후륜의 tan차이로 계산
        angular_velocity = (avg_velocity / wheelbase_) * (std::tan(delta_f) - std::tan(delta_r));
    }
    
    //Odometry 적분 (Euler method)
    dr_x_ += avg_velocity * std::cos(dr_theta_) * dt;
    dr_y_ += avg_velocity * std::sin(dr_theta_) * dt;
    dr_theta_ += angular_velocity * dt;
    
    //Normalize theta(-PI ~ PI)
    while (dr_theta_ > M_PI) dr_theta_ -= 2.0 * M_PI;
    while (dr_theta_ < -M_PI) dr_theta_ += 2.0 * M_PI;
    
    //Odometry 메시지 발행
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    //Position
    odom_msg.pose.pose.position.x = dr_x_;
    odom_msg.pose.pose.position.y = dr_y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    //Orientation (quaternion from yaw)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dr_theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
    //Velocity
    odom_msg.twist.twist.linear.x = avg_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;
    
    odom_pub_->publish(odom_msg);
    
    //update
    prev_encoder_wheel_ = encoder_wheel_;
    prev_time_ = current_time;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "[Localizer] Dead Reckoning: (%.2f, %.2f, %.2f°) | v=%.2f m/s",
                        dr_x_, dr_y_, dr_theta_ * 180.0 / M_PI, avg_velocity);
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pagv_localizer::LocalizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
