//
// Created by sub on 9/17/18.
//

#ifndef LIZI_HW_VELOCITIES_LPF_H
#define LIZI_HW_VELOCITIES_LPF_H

#include <lpf_ros/lpf_ros.h>
#include "wheel.h"

class VelocitiesLpf
{
private:

    std::vector<wheel*> wheels_;

    std::vector<lpf::Lpf> filters_;

public:
    void init(ros::NodeHandle &nh, std::vector<wheel*> & wheels);

    void update();
};


#endif //LIZI_HW_VELOCITIES_LPF_H
