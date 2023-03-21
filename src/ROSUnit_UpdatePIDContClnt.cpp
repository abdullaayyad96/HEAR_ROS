#include "HEAR_ROS/ROSUnit_UpdatePIDContClnt.hpp"

namespace HEAR{

ROSUnit_UpdatePIDContClnt::ROSUnit_UpdatePIDContClnt(ros::NodeHandle& t_main_handler, std::string Drone_Name, std::string Channel) : nh_(t_main_handler){

    m_client_pid = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/" + Channel);
    
}

ROSUnit_UpdatePIDContClnt::~ROSUnit_UpdatePIDContClnt() {

}

bool ROSUnit_UpdatePIDContClnt::process(BaseMsg* t_msg) {
    PID_UpdateMsg* _update_msg = (PID_UpdateMsg*)t_msg;
    hear_msgs::Update_Controller_PID srv;
    srv.request.controller_parameters.id = (int)_update_msg->param.id;
    srv.request.controller_parameters.pid_kp = _update_msg->param.kp;
    srv.request.controller_parameters.pid_ki = _update_msg->param.ki;
    srv.request.controller_parameters.pid_kd = _update_msg->param.kd;
    srv.request.controller_parameters.pid_kdd = _update_msg->param.kdd;
    srv.request.controller_parameters.pid_anti_windup = _update_msg->param.anti_windup;
    srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->param.en_pv_derivation;
    srv.request.controller_parameters.pid_dt = _update_msg->param.dt;
    bool success = false;

    success = m_client_pid.call(srv);
    
    if (success) {
        ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
        return true;
    }
    else {
        ROS_ERROR("Failed to call service /update_controller/pid");
        return false;
    }
    
}

}
