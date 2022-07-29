#include "HEAR_ROS/ROSUnit_FloatClnt.hpp"

namespace HEAR{

ROSUnitFloatClient::ROSUnitFloatClient(ros::NodeHandle &nh, std::string t_name){
    m_client = nh.serviceClient<hear_msgs::set_float>(t_name);
}

void ROSUnitFloatClient::process(DataMsg* t_msg){
    hear_msgs::set_float t_srv; 
    t_srv.request.data = ((FloatMsg*) t_msg)->data;   
    m_client.call(t_srv);
}

}