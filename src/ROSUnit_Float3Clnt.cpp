#include "HEAR_ROS/ROSUnit_Float3Clnt.hpp"

namespace HEAR{

ROSUnitFloat3Client::ROSUnitFloat3Client(ros::NodeHandle &nh, std::string t_name) : nh_(nh){
    m_client = nh_.serviceClient<hear_msgs::set_point>(t_name);
}

bool ROSUnitFloat3Client::process(){
    hear_msgs::set_point msg; 
    msg.request.x = this->data[0]; 
    msg.request.y = this->data[1];
    msg.request.z = this->data[2];    
    return m_client.call(msg);
}

}