#ifndef ROSUNIT_BOOLSRV_HPP
#define ROSUNIT_BOOLSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_bool.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_BoolServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    ExternalTrigger<BaseMsg>* ext_trig;
    bool srv_callback(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
public:
    ROSUnit_BoolServer(ros::NodeHandle&);
    ExternalTrigger<BaseMsg>* registerServer(const std::string&);
    
};

}

#endif