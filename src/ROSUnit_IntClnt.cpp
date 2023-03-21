#include "HEAR_ROS/ROSUnit_IntClnt.hpp"

namespace HEAR{

ROSUnitIntClient::ROSUnitIntClient(ros::NodeHandle &nh, std::string t_name): nh_(nh){
    m_client = nh_.serviceClient<hear_msgs::set_int>(t_name);
}

bool ROSUnitIntClient::process(){
    hear_msgs::set_int msg;
    msg.request.data = this->data;    
    return m_client.call(msg);
}

}