#ifndef ROSUNIT_UPDATETRAJECTORYSRV_HPP
#define ROSUNIT_UPDATETRAJECTORYSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/TRAJECTORY_param.h>
#include <hear_msgs/Update_Trajectory.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_UpdateTrajectorySrv {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    ExternalTrigger<BaseMsg>* ext_trig;
    bool srv_callback(hear_msgs::Update_Trajectory::Request&, hear_msgs::Update_Trajectory::Response&);
public:
    ROSUnit_UpdateTrajectorySrv(ros::NodeHandle& nh) : nh_(nh) {}
    ExternalTrigger<BaseMsg>* registerServer(const std::string&);
    
};

}

#endif