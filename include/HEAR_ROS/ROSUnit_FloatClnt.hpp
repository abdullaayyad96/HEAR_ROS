#ifndef ROSUNIT_FLOATCLNT
#define ROSUNIT_FLOATCLNT

#include <ros/ros.h>
#include <hear_msgs/set_float.h>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_ROS/ROSUnit_Client.hpp"
#include <string>

namespace HEAR{

class ROSUnitFloatClient : public ROSUnit_Client<float>{
public:
    ros::NodeHandle nh_;
    ROSUnitFloatClient(ros::NodeHandle&, std::string);
    bool process();
};

}

#endif