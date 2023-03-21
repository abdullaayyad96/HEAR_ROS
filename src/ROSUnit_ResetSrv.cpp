#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"

namespace HEAR{

ROSUnit_ResetServer::ROSUnit_ResetServer(ros::NodeHandle &nh) {
    ext_trig = new ExternalTrigger<BaseMsg>;

}

ExternalTrigger<BaseMsg>* ROSUnit_ResetServer::registerServer(const std::string &service_topic){
    m_server = nh_.advertiseService(service_topic, &ROSUnit_ResetServer::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_ResetServer::srv_callback(hear_msgs::set_int::Request& req, hear_msgs::set_int::Response& res){
    IntMsg msg;
    msg.data = req.data;
    ext_trig->UpdateCallback((BaseMsg*)&msg);
    return true;
}

}