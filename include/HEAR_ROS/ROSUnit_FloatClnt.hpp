#ifndef ROSUNIT_FLOATCLNT
#define ROSUNIT_FLOATCLNT

#include <ros/ros.h>
#include <hear_msgs/set_float.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>

namespace HEAR{

class ROSUnitFloatClient {
private:
    ros::ServiceClient m_client;
public:
    ROSUnitFloatClient(ros::NodeHandle&, std::string);
    bool process(float data);
};

}

#endif