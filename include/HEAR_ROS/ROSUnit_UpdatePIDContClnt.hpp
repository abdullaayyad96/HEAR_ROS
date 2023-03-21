#ifndef ROSUNIT_UPDATEPIDCONTCLNT_HPP
#define ROSUNIT_UPDATEPIDCONTCLNT_HPP


#include <ros/ros.h>
#include <hear_msgs/PID_param.h>
#include <hear_msgs/Update_Controller_PID.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>



namespace HEAR{
class ROSUnit_UpdatePIDContClnt {
private:
    ros::NodeHandle nh_;
    ros::ServiceClient m_client_pid;
public:
    bool process(BaseMsg* t_msg);
    ROSUnit_UpdatePIDContClnt(ros::NodeHandle& nh, std::string Drone_Name, std::string Channel);
    ~ROSUnit_UpdatePIDContClnt();
    
};

}

#endif