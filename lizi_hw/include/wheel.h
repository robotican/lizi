/* Author: Elhay Rauper*/


#ifndef LIZI_HW_WHEEL_H
#define LIZI_HW_WHEEL_H

struct wheel
{
    double pos = 0;
    double vel = 0;
    double effort = 0;
    double timestamp = 0;
    double cmd_vel = 0;
};

#endif //LIZI_HW_WHEEL_H
