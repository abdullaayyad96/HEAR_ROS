#ifndef ROSUNIT_BOOLCLNT
#define ROSUNIT_BOOLCLNT

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_ROS/ROSUnit_Client.hpp"
#include <string>

namespace HEAR{

class ROSUnitBoolClient: public ROSUnit_Client{

public:
    ROSUnitBoolClient(ros::NodeHandle&, std::string);
    bool process(bool data);
};

}

#endif