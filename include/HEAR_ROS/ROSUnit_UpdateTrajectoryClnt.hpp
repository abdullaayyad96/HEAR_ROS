#ifndef ROSUNIT_UPDATETRAJECTORYCLNT_HPP
#define ROSUNIT_UPDATETRAJECTORYCLNT_HPP


#include <ros/ros.h>
#include <hear_msgs/Update_Trajectory.h>
#include "HEAR_core/DataTypes.hpp"
#include <string>



namespace HEAR{
class ROSUnit_UpdateTrajectoryClnt {
private:
    ros::NodeHandle nh_;
    ros::ServiceClient m_client;
public:
    bool process(BaseMsg* t_msg);
    ROSUnit_UpdateTrajectoryClnt(ros::NodeHandle& nh, std::string Drone_Name);
    ~ROSUnit_UpdateTrajectoryClnt();
    
};

}

#endif