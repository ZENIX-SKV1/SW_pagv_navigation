#include "pagv_motion_controller/motion_controller_node.hpp"
#include <algorithm>

namespace pagv_motion_controller {

MotionControllerNode::MotionControllerNode()
    : Node("motion_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "[MotionController] Initializing...");
    
    // Publishers - Isaac Sim으로 직접 전송
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/isaac/steering_cmd", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/isaac/velocity_cmd", 10);
    
    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MotionControllerNode::cmd_vel_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "[MotionController] Ready");
    RCLCPP_INFO(this->get_logger(), "  - 8-Wheel 4-Axle All-Wheel Steering");
    RCLCPP_INFO(this->get_logger(), "  - Max steering angle: 33°");
    RCLCPP_INFO(this->get_logger(), "  - Publishing to Isaac Sim:");
    RCLCPP_INFO(this->get_logger(), "    /isaac/steering_cmd [4 floats]");
    RCLCPP_INFO(this->get_logger(), "    /isaac/velocity_cmd [2 floats]");
}

void MotionControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    
    // 8륜 4축 제어 계산 (전 축 조향 + ICR 일치)
    std::array<double, 4> steering_angles;
    std::array<double, 2> wheel_speeds;
    
    compute_8wheel_control(linear_vel, angular_vel, steering_angles, wheel_speeds);
    
    // Isaac Sim으로 직접 발행
    auto steering_msg = std_msgs::msg::Float32MultiArray();
    steering_msg.data.resize(4);
    for (size_t i = 0; i < 4; ++i) {
        steering_msg.data[i] = static_cast<float>(steering_angles[i]);
    }
    steering_pub_->publish(steering_msg);
    
    auto velocity_msg = std_msgs::msg::Float32MultiArray();
    velocity_msg.data.resize(2);
    velocity_msg.data[0] = static_cast<float>(wheel_speeds[0]);
    velocity_msg.data[1] = static_cast<float>(wheel_speeds[1]);
    velocity_pub_->publish(velocity_msg);
    
    // Log (throttled)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[MotionController] v=%.2f m/s, w=%.2f rad/s | Steer: [%.1f, %.1f, %.1f, %.1f]°",
                        linear_vel, angular_vel,
                        steering_angles[0] * 180.0 / M_PI,
                        steering_angles[1] * 180.0 / M_PI,
                        steering_angles[2] * 180.0 / M_PI,
                        steering_angles[3] * 180.0 / M_PI);
}

void MotionControllerNode::compute_8wheel_control(double linear_vel, double angular_vel,
                                                   std::array<double, 4>& steering_angles,
                                                   std::array<double, 2>& wheel_speeds)
{
    if (std::abs(angular_vel) < 0.001) {
        // 직진 - 모든 축 조향각 0
        steering_angles = {0.0, 0.0, 0.0, 0.0};
        wheel_speeds = {linear_vel, linear_vel};
        return;
    }
    
    //ICR 계산
    double turning_radius = std::abs(linear_vel / angular_vel);
    
    // 차량 중심에서 각 축까지의 거리
    // L1: 1축-중심, L2: 2축-중심, L3: 3축-중심, L4: 4축-중심
    double L1 = config_.wheelbase_l1;  // 2.5m (1축 → 중심)
    double L2 = config_.wheelbase_l2;  // 1.5m (2축 → 중심)
    double L3 = config_.wheelbase_l3;  // 2.5m (3축 → 중심)
    // L4는 대칭이므로 L1과 동일하게 간주 (4축은 후방)
    
    double track_width = config_.track_width;  // 2.0m
    
    //각 축의 좌우 바퀴별 ICR까지의 거리 계산
    // 좌회전(angular_vel > 0): ICR은 좌측, 우회전: ICR은 우측
    bool left_turn = (angular_vel > 0);
    
    //ICR 위치: 차량 중심에서 좌측(+) 또는 우측(-)으로 turning_radius만큼
    double icr_y = left_turn ? turning_radius : -turning_radius;
    
    //각 축의 좌우 바퀴 위치 (차량 중심 기준)
    struct WheelPos {
        double x;  // 전후 위치 (전방 +)
        double y;  // 좌우 위치 (좌측 +)
    };
    
    //1축 (전방) - Left/Right
    WheelPos axle1_left  = {L1, track_width / 2.0};
    WheelPos axle1_right = {L1, -track_width / 2.0};
    
    //2축 (전방 중간) - Left/Right
    WheelPos axle2_left  = {L2, track_width / 2.0};
    WheelPos axle2_right = {L2, -track_width / 2.0};
    
    //3축 (후방 중간) - Left/Right
    WheelPos axle3_left  = {-L3, track_width / 2.0};
    WheelPos axle3_right = {-L3, -track_width / 2.0};
    
    //4축 (후방) - Left/Right
    WheelPos axle4_left  = {-L1, track_width / 2.0};
    WheelPos axle4_right = {-L1, -track_width / 2.0};
    
    //각 바퀴에서 ICR까지의 거리 및 조향각 계산
    auto calc_steering = [&](const WheelPos& wheel) -> double {
        // ICR 좌표: (0, icr_y)
        double dx = 0.0 - wheel.x;  // ICR.x - wheel.x
        double dy = icr_y - wheel.y; // ICR.y - wheel.y
        
        // 조향각 = atan2(dx, dy)
        // (차량 전방이 +x이므로, atan2(전후 거리, 좌우 거리))
        return std::atan2(dx, dy);
    };
    
    double steer1_left  = calc_steering(axle1_left);
    double steer1_right = calc_steering(axle1_right);
    double steer2_left  = calc_steering(axle2_left);
    double steer2_right = calc_steering(axle2_right);
    double steer3_left  = calc_steering(axle3_left);
    double steer3_right = calc_steering(axle3_right);
    double steer4_left  = calc_steering(axle4_left);
    double steer4_right = calc_steering(axle4_right);
    
    //각 축의 대표 조향각 (좌우 평균)
    double steer1 = (steer1_left + steer1_right) / 2.0;
    double steer2 = (steer2_left + steer2_right) / 2.0;
    double steer3 = (steer3_left + steer3_right) / 2.0;
    double steer4 = (steer4_left + steer4_right) / 2.0;
    
    //조향각 제한 (±33° = ±0.5759 rad)
    const double max_steer = config_.max_steering_angle;  // 0.785 rad (45°) → 33° = 0.5759 rad로 변경 필요
    
    steer1 = std::clamp(steer1, -max_steer, max_steer);
    steer2 = std::clamp(steer2, -max_steer, max_steer);
    steer3 = std::clamp(steer3, -max_steer, max_steer);
    steer4 = std::clamp(steer4, -max_steer, max_steer);
    
    //출력 (1축, 2축, 3축, 4축)
    steering_angles[0] = steer1;
    steering_angles[1] = steer2;
    steering_angles[2] = steer3;
    steering_angles[3] = steer4;
    
    //구동축 속도 (2축, 3축만 구동)
    // 2축과 3축의 속도를 ICR 거리 비율로 조정(isaac sim 테스트 하면서 선택적 반영하자)
    // 여기서는 단순히 linear_vel 사용
    wheel_speeds[0] = linear_vel;  // 2축
    wheel_speeds[1] = linear_vel;  // 3축
}

} 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pagv_motion_controller::MotionControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
