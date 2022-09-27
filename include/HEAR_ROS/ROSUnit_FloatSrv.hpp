#ifndef ROSUNIT_FLOATSRV_HPP
#define ROSUNIT_FLOATSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_float.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_FloatServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    UpdateTrigger* ext_trig;
    bool srv_callback(hear_msgs::set_float::Request&, hear_msgs::set_float::Response&);
public:
    ROSUnit_FloatServer(ros::NodeHandle&);
    UpdateTrigger* registerServer(const std::string&);
    
};

}

#endif