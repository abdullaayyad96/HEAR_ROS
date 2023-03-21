#include "HEAR_ROS/ROSUnit_UpdateBoundingContClnt.hpp"

namespace HEAR{

ROSUnit_UpdateBoundingContClnt::ROSUnit_UpdateBoundingContClnt(ros::NodeHandle& t_main_handler, std::string Drone_Name, std::string Channel) : nh_(t_main_handler){

    m_client_bounding = nh_.serviceClient<hear_msgs::Update_Controller_Bounding>(Drone_Name + "update_controller/bounding/" + Channel);

}

ROSUnit_UpdateBoundingContClnt::~ROSUnit_UpdateBoundingContClnt() {

}

bool ROSUnit_UpdateBoundingContClnt::process(BaseMsg* t_msg) {
    BoundingCtrl_UpdateMsg* _update_msg = (BoundingCtrl_UpdateMsg*)t_msg;
    hear_msgs::Update_Controller_Bounding srv;
    srv.request.controller_parameters.id = (int)_update_msg->param.id;
    srv.request.controller_parameters.bounding_eps_1 = _update_msg->param.eps_1;
    srv.request.controller_parameters.bounding_eps_2 = _update_msg->param.eps_2;
    srv.request.controller_parameters.bounding_h_o1 = _update_msg->param.h_o1;
    srv.request.controller_parameters.bounding_h_o2 = _update_msg->param.h_o2;
    bool success = false;

    success = m_client_bounding.call(srv);

    if (success) {
        ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
        return true;
    }
    else {
        ROS_ERROR("Failed to call service /update_controller/bounding/");
        return false;
    }
}

}
