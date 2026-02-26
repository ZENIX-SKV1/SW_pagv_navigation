#ifndef PAGV_AMR_CORE__NAVIGATION__PATH_FOLLOWER_HPP_
#define PAGV_AMR_CORE__NAVIGATION__PATH_FOLLOWER_HPP_

#include <geometry_msgs/msg/twist.hpp>

namespace pagv_amr_core 
{

class PathFollower 
{
public:
    PathFollower() = default;
    
    geometry_msgs::msg::Twist compute_velocity(double x, double y, double theta,
                                                 double goal_x, double goal_y);
};

} 

#endif
