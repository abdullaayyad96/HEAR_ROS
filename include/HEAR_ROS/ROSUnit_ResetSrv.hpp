
#ifndef ROSUNIT_RESETSRV_HPP
#define ROSUNIT_RESETSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_int.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_ResetServer {
private:
    ros::NodeHandle nh_;
    ExternalTrigger<BaseMsg>* ext_trig;
    ros::ServiceServer m_server;
    bool srv_callback(hear_msgs::set_int::Request&, hear_msgs::set_int::Response&);
public:
    ROSUnit_ResetServer(ros::NodeHandle&);
    ExternalTrigger<BaseMsg>* registerServer(const std::string&);
    
 
};

}

#endif