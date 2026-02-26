#ifndef PAGV_AMR_CORE__NAVIGATION__ARRIVAL_DETECTOR_HPP_
#define PAGV_AMR_CORE__NAVIGATION__ARRIVAL_DETECTOR_HPP_

namespace pagv_amr_core 
{

class ArrivalDetector 
{
public:
    ArrivalDetector() = default;
    
    bool is_arrived(double x, double y, double theta,
                    double goal_x, double goal_y, double goal_theta,
                    double pos_tol = 0.5, double ang_tol = 0.1);
};

} 

#endif
