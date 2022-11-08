#ifndef ROSUNIT_UPDATECONTCLNT_HPP
#define ROSUNIT_UPDATECONTCLNT_HPP


#include <ros/ros.h>
#include <hear_msgs/PID_param.h>
#include <hear_msgs/MRFT_param.h>
#include <hear_msgs/Update_Controller_PID.h>
#include <hear_msgs/Update_Controller_MRFT.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>



namespace HEAR{
class ROSUnit_UpdateContClnt {
private:
    ros::NodeHandle nh_;
    ros::ServiceClient m_client_pid_x;
    ros::ServiceClient m_client_pid_y;
    ros::ServiceClient m_client_pid_z;
    ros::ServiceClient m_client_pid_roll;
    ros::ServiceClient m_client_pid_pitch;
    ros::ServiceClient m_client_pid_yaw;
    ros::ServiceClient m_client_pid_yaw_rate;
    ros::ServiceClient m_client_mrft_x;
    ros::ServiceClient m_client_mrft_y;
    ros::ServiceClient m_client_mrft_z;
    ros::ServiceClient m_client_mrft_roll;
    ros::ServiceClient m_client_mrft_pitch;
    ros::ServiceClient m_client_mrft_yaw;
    ros::ServiceClient m_client_mrft_yaw_rate;
public:
    bool process(UpdateMsg* t_msg, BLOCK_ID ID);
    ROSUnit_UpdateContClnt(ros::NodeHandle& nh, std::string Drone_Name);
    ~ROSUnit_UpdateContClnt();
    
};

}

#endif