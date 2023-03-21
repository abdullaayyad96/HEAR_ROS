#ifndef ROSUNIT_UPDATEMRFTCONTCLNT_HPP
#define ROSUNIT_UPDATEMRFTCONTCLNT_HPP


#include <ros/ros.h>
#include <hear_msgs/MRFT_param.h>
#include <hear_msgs/Update_Controller_MRFT.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>



namespace HEAR{
class ROSUnit_UpdateMRFTContClnt {
private:
    ros::NodeHandle nh_;
    ros::ServiceClient m_client_mrft;
public:
    bool process(BaseMsg* t_msg);
    ROSUnit_UpdateMRFTContClnt(ros::NodeHandle& nh, std::string Drone_Name, std::string Channel);
    ~ROSUnit_UpdateMRFTContClnt();
    
};

}

#endif