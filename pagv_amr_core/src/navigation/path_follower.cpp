#include "pagv_amr_core/navigation/path_follower.hpp"
#include <cmath>

namespace pagv_amr_core 
{

geometry_msgs::msg::Twist PathFollower::compute_velocity(double x, double y, double theta,
                                                          double goal_x, double goal_y)
{
    geometry_msgs::msg::Twist cmd_vel;
    
    double dx = goal_x - x;
    double dy = goal_y - y;
    double goal_angle = std::atan2(dy, dx);
    double angle_error = goal_angle - theta;
    
    //Normalize angle
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    
    cmd_vel.linear.x = 1.0;  //constant speed
    cmd_vel.angular.z = 2.0 * angle_error;  //P controller
    
    return cmd_vel;
}

}
