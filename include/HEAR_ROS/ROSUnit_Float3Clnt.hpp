#ifndef ROSUNIT_FLOAT3CLNT
#define ROSUNIT_FLOAT3CLNT

#include <ros/ros.h>
#include <hear_msgs/set_point.h>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_ROS/ROSUnit_Client.hpp"
#include <string>

namespace HEAR{

class ROSUnitFloat3Client : public ROSUnit_Client<std::vector<float>>{
public:
    ros::NodeHandle nh_;
    ROSUnitFloat3Client(ros::NodeHandle&, std::string);
    bool process();
};

}

#endif