#ifndef ROSUNIT_INTSRV_HPP
#define ROSUNIT_INTSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_int.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_IntServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    ExternalTrigger<BaseMsg>* ext_trig;
    bool srv_callback(hear_msgs::set_int::Request&, hear_msgs::set_int::Response&);
public:
    ROSUnit_IntServer(ros::NodeHandle&);
    ExternalTrigger<BaseMsg>* registerServer(const std::string&);
    
};

}

#endif