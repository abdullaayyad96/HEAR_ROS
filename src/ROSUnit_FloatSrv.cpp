#include "HEAR_ROS/ROSUnit_FloatSrv.hpp"

namespace HEAR{

ROSUnit_FloatServer::ROSUnit_FloatServer(ros::NodeHandle &nh) : nh_(nh){
    ext_trig = new ExternalTrigger<BaseMsg>;
}

ExternalTrigger<BaseMsg>* ROSUnit_FloatServer::registerServer(const std::string &service_topic){
    m_server = nh_.advertiseService(service_topic, &ROSUnit_FloatServer::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_FloatServer::srv_callback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res){
    FloatMsg msg;
    msg._value = req.data;
    ext_trig->UpdateCallback((BaseMsg*)&msg);
    return true;
}

}