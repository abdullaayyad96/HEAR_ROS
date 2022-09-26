#ifndef ROSUNIT_CLIENT_HPP
#define ROSUNIT_CLIENT_HPP

#include <ros/ros.h>
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

template <typename T>
class ROSUnit_Client {
public:
    T data;
    ros::ServiceClient m_client;
    virtual bool process() {return true;};
};
}

#endif