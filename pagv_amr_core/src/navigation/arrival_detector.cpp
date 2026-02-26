#include "pagv_amr_core/navigation/arrival_detector.hpp"
#include <cmath>

namespace pagv_amr_core {

bool ArrivalDetector::is_arrived(double x, double y, double theta,
                                 double goal_x, double goal_y, double goal_theta,
                                 double pos_tol, double ang_tol)
{
    double dx = goal_x - x;
    double dy = goal_y - y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    double angle_diff = std::abs(goal_theta - theta);
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    angle_diff = std::abs(angle_diff);
    
    return (distance < pos_tol) && (angle_diff < ang_tol);
}

} 
