#ifndef ROSUNIT_CLIENT_HPP
#define ROSUNIT_CLIENT_HPP

#include <ros/ros.h>
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

class ROSUnit_Client {
protected:
    ros::ServiceClient m_client;
};
}

#endif