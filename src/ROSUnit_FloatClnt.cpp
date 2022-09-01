#include "HEAR_ROS/ROSUnit_FloatClnt.hpp"

namespace HEAR{

ROSUnitFloatClient::ROSUnitFloatClient(ros::NodeHandle &nh, std::string t_name) : nh_(nh){
    m_client = nh_.serviceClient<hear_msgs::set_float>(t_name);
}

bool ROSUnitFloatClient::process(float data){
    hear_msgs::set_float msg; 
    msg.request.data = data;   
    return m_client.call(msg);
}

}