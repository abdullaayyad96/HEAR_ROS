#ifndef ROSUNIT_FLOAT3SRV_HPP
#define ROSUNIT_FLOAT3SRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_point.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_Float3Server {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    ExternalTrigger<BaseMsg>* ext_trig;
    bool srv_callback(hear_msgs::set_point::Request&, hear_msgs::set_point::Response&);
public:
    ROSUnit_Float3Server(ros::NodeHandle&);
    ExternalTrigger<BaseMsg>* registerServer(const std::string&);
    
};

}

#endif