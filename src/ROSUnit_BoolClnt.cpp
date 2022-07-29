#include "HEAR_ROS/ROSUnit_BoolClnt.hpp"

namespace HEAR{

ROSUnitBoolClient::ROSUnitBoolClient(ros::NodeHandle &nh, std::string t_name){
    m_client = nh.serviceClient<hear_msgs::set_bool>(t_name);
}

void ROSUnitBoolClient::process(DataMsg* t_msg){
    hear_msgs::set_bool t_srv;
    t_srv.request.data = ((BoolMsg*) t_msg)->data;    
    m_client.call(msg);
}

}