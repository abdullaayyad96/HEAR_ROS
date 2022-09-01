#include "HEAR_ROS/ROSUnit_BoolClnt.hpp"

namespace HEAR{

ROSUnitBoolClient::ROSUnitBoolClient(ros::NodeHandle &nh, std::string t_name): nh_(nh){
    m_client = nh_.serviceClient<std_srvs::SetBool>(t_name);
}

bool ROSUnitBoolClient::process(bool data){
    std_srvs::SetBool msg;
    msg.request.data = data;    
    return m_client.call(msg);
}

}