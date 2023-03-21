#ifndef ROSUNIT_INTCLNT
#define ROSUNIT_INTCLNT

#include <ros/ros.h>
#include "HEAR_core/DataTypes.hpp"
#include <hear_msgs/set_int.h>
#include "HEAR_ROS/ROSUnit_Client.hpp"
#include <string>

namespace HEAR{

class ROSUnitIntClient: public ROSUnit_Client<int>{

public:
    ros::NodeHandle nh_;
    ROSUnitIntClient(ros::NodeHandle& nh, std::string);
    bool process();
};

}

#endif