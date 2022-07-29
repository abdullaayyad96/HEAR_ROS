#ifndef ROSUNIT_BOOLCLNT
#define ROSUNIT_BOOLCLNT

#include <ros/ros.h>
#include <hear_msgs/set_bool.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>

namespace HEAR{

class ROSUnitBoolClient {
private:
    ros::ServiceClient m_client;
public:
    ROSUnitBoolClient(ros::NodeHandle&, std::string);
    void process(DataMsg* t_msg);
};

}

#endif