#ifndef ROSUNIT_UPDATEBOUNDINGSRV_HPP
#define ROSUNIT_UPDATEBOUNDINGSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/PID_param.h>
#include <hear_msgs/Update_Controller_Bounding.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_UpdateBOUNDINGsrv {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    ExternalTrigger<BaseMsg>* ext_trig;
    bool srv_callback(hear_msgs::Update_Controller_Bounding::Request&, hear_msgs::Update_Controller_Bounding::Response&);
public:
    ROSUnit_UpdateBOUNDINGsrv(ros::NodeHandle& nh) : nh_(nh) {}
    ExternalTrigger<BaseMsg>* registerServer(const std::string&);
    
};

}

#endif