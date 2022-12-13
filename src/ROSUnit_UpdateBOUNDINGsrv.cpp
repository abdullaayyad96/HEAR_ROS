#include "HEAR_ROS/ROSUnit_UpdateBOUNDINGsrv.hpp"

namespace HEAR{

UpdateTrigger* ROSUnit_UpdateBOUNDINGsrv::registerServer(const std::string &service_topic){
    ext_trig = new UpdateTrigger;
    this->m_server = this->nh_.advertiseService(service_topic, &ROSUnit_UpdateBOUNDINGsrv::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_UpdateBOUNDINGsrv::srv_callback(hear_msgs::Update_Controller_Bounding::Request& req, hear_msgs::Update_Controller_Bounding::Response& res){
    BoundingCtrl_UpdateMsg msg;
    msg.param.id = (int)req.controller_parameters.id;
    msg.param.eps_1 = req.controller_parameters.bounding_eps_1;
    msg.param.eps_2 = req.controller_parameters.bounding_eps_2 ;
    msg.param.h_o1 = req.controller_parameters.bounding_h_o1;
    msg.param.h_o2 = req.controller_parameters.bounding_h_o2;
    ext_trig->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

}