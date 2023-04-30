#include "HEAR_ROS/ROSUnit_Float3Srv.hpp"

namespace HEAR{

ROSUnit_Float3Server::ROSUnit_Float3Server(ros::NodeHandle &nh) : nh_(nh){
    ext_trig = new ExternalTrigger<BaseMsg>;
}

ExternalTrigger<BaseMsg>* ROSUnit_Float3Server::registerServer(const std::string &service_topic){
    m_server = nh_.advertiseService(service_topic, &ROSUnit_Float3Server::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_Float3Server::srv_callback(hear_msgs::set_point::Request& req, hear_msgs::set_point::Response& res){
    Float3Msg msg;
    msg.data[0] = req.x;
    msg.data[1] = req.y;
    msg.data[2] = req.z;
    ext_trig->UpdateCallback((BaseMsg*)&msg);
    return true;
}

}