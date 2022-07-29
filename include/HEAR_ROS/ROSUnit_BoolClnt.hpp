#ifndef ROSUNIT_BOOLCLNT
#define ROSUNIT_BOOLCLNT

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>

namespace HEAR{

class ROSUnitBoolClient {
private:
    ros::ServiceClient m_client;
public:
    ROSUnitBoolClient(ros::NodeHandle&, std::string);
    bool process(bool data);
};

}

#endif