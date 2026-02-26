#ifndef PAGV_AMR_CORE__NAVIGATION__NAVIGATION_MANAGER_HPP_
#define PAGV_AMR_CORE__NAVIGATION__NAVIGATION_MANAGER_HPP_

#include <pagv_amr_core/node_edge_info.h>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <vector>

namespace pagv_amr_core 
{

struct NavigationConfig 
{
    double lookahead_distance{2.0};
    double max_speed{1.0};
    double min_speed{0.1};
    double position_tolerance{0.5};
    double orientation_tolerance{0.1};
};

class NavigationManager 
{
public:
    NavigationManager();
    explicit NavigationManager(const NavigationConfig& config);
    
    void set_goal(const NodePosition& goal);

    void set_arc_goal(double start_x, double start_y, double start_theta,
                     double target_x, double target_y,
                     double center_x, double center_y);

    void set_current_pose(const Pose& pose);
    
    geometry_msgs::msg::Twist compute_velocity_command();
    
    bool is_goal_reached() const;
    double get_distance_to_goal() const;
    
private:
    double normalize_angle(double angle) const;
    double compute_linear_velocity() const;
    
    double calculate_pure_pursuit(double current_x, double current_y, double current_theta,
                                  double target_x, double target_y, double lookahead_dist);

    void compute_straight_velocity(double& linear, double& angular);
    
    void compute_arc_velocity(double& linear, double& angular);                                  
    
    void apply_radius_error_correction(double radius_error, 
                                       double& linear_speed, double& angular_speed);
    
    NavigationConfig config_;
    Pose current_pose_;
    Pose goal_pose_;
    bool has_goal_{false};
    
    bool use_arc_{false};
    double target_x_{0.0};
    double target_y_{0.0};
    double arc_center_x_{0.0};
    double arc_center_y_{0.0};
    double arc_radius_{0.0};
    double arc_start_angle_{0.0};
    double arc_end_angle_{0.0};
    bool arc_clockwise_{false};
    bool goal_reached_{false};
};

} 

#endif
