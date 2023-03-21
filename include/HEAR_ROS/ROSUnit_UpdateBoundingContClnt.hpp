#ifndef ROSUNIT_UPDATEBOUNDINGCONTCLNT_HPP
#define ROSUNIT_UPDATEBOUNDINGCONTCLNT_HPP


#include <ros/ros.h>
#include <hear_msgs/Update_Controller_Bounding.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>



namespace HEAR{
class ROSUnit_UpdateBoundingContClnt {
private:
    ros::NodeHandle nh_;
    ros::ServiceClient m_client_bounding;
public:
    bool process(BaseMsg* t_msg);
    ROSUnit_UpdateBoundingContClnt(ros::NodeHandle& nh, std::string Drone_Name, std::string Channel);
    ~ROSUnit_UpdateBoundingContClnt();
    
};

}

#endif