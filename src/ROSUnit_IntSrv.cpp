#include "HEAR_ROS/ROSUnit_IntSrv.hpp"

namespace HEAR{

ROSUnit_IntServer::ROSUnit_IntServer(ros::NodeHandle &nh) : nh_(nh){
    ext_trig = new ExternalTrigger<BaseMsg>;
}

ExternalTrigger<BaseMsg>* ROSUnit_IntServer::registerServer(const std::string &service_topic){
    m_server = nh_.advertiseService(service_topic, &ROSUnit_IntServer::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_IntServer::srv_callback(hear_msgs::set_int::Request& req, hear_msgs::set_int::Response& res){
    IntMsg msg;
    msg.data = req.data;
    ext_trig->UpdateCallback((BaseMsg*)&msg);
    return true;
}

}