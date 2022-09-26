#include "HEAR_ROS/ROSUnit_BoolClnt.hpp"

namespace HEAR{

ROSUnitBoolClient::ROSUnitBoolClient(ros::NodeHandle &nh, std::string t_name): nh_(nh){
    m_client = nh_.serviceClient<hear_msgs::set_bool>(t_name);
}

bool ROSUnitBoolClient::process(){
    hear_msgs::set_bool msg;
    msg.request.data = this->data;    
    return m_client.call(msg);
}

}