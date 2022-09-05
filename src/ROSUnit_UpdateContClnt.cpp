#include "HEAR_ROS/ROSUnit_UpdateContClnt.hpp"

namespace HEAR{

ROSUnit_UpdateContClnt::ROSUnit_UpdateContClnt(ros::NodeHandle& t_main_handler) : nh_(t_main_handler){

    // PID of the inner loop
    m_client_pid_x = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/x");
    m_client_pid_y = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/y");
    m_client_pid_z = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/z");

    // PID of the outer loop
    m_client_pid_roll = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/roll");
    m_client_pid_pitch = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/pitch");
    m_client_pid_yaw = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/yaw");
    m_client_pid_yaw_rate = nh_.serviceClient<hear_msgs::Update_Controller_PID>("update_controller/pid/yaw_rate");

    // MRFT of the inner loop
    m_client_mrft_x = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/x");
    m_client_mrft_y = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/y");
    m_client_mrft_z = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/z");

    // MRFT of the outer loop
    m_client_mrft_roll = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/roll");
    m_client_mrft_pitch = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/pitch");
    m_client_mrft_yaw = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/yaw");
    m_client_mrft_yaw_rate = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>("update_controller/mrft/yaw_rate");
}   

ROSUnit_UpdateContClnt::~ROSUnit_UpdateContClnt() {

}

void ROSUnit_UpdateContClnt::process(UpdateMsg* t_msg, BLOCK_ID ID) {
    //TODO: Write this function
    if(ID == PID) {
        PID_UpdateMsg* _update_msg = (PID_UpdateMsg*)t_msg;
        hear_msgs::Update_Controller_PID srv;
        srv.request.controller_parameters.id = static_cast<int>(_update_msg->param.id);
        srv.request.controller_parameters.pid_kp = _update_msg->param.kp;
        srv.request.controller_parameters.pid_ki = _update_msg->param.ki;
        srv.request.controller_parameters.pid_kd = _update_msg->param.kd;
        srv.request.controller_parameters.pid_kdd = _update_msg->param.kdd;
        srv.request.controller_parameters.pid_anti_windup = _update_msg->param.anti_windup;
        srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->param.en_pv_derivation;
        srv.request.controller_parameters.pid_dt = _update_msg->param.dt;
        bool success1 = false;
        bool success2 = false;
        bool success3 = false; 
        bool success4 = false;
        if( (int)_update_msg->param.id <= (int)PID_ID::PID_Z){
            success1 = m_client_pid_x.call(srv);
            success2 = m_client_pid_y.call(srv);
            success3 = m_client_pid_z.call(srv);
            success4 = true;
        }else{
            success1 = m_client_pid_roll.call(srv);
            success2 = m_client_pid_pitch.call(srv);
            success3 = m_client_pid_yaw.call(srv);
            success4 = m_client_pid_yaw_rate.call(srv);
        }
        if (success1 && success2 && success3 && success4) {
            ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
        }
        else {
            ROS_ERROR("Failed to call service /update_controller");
        }
    }
    else if(ID == MRFT) {
        MRFT_UpdateMsg* _update_msg = (MRFT_UpdateMsg*)t_msg;
        hear_msgs::Update_Controller_MRFT srv;
        srv.request.controller_parameters.id = static_cast<int>(_update_msg->param.id);
        srv.request.controller_parameters.mrft_beta = _update_msg->param.beta;
        srv.request.controller_parameters.mrft_relay_amp = _update_msg->param.relay_amp;
        srv.request.controller_parameters.mrft_bias = _update_msg->param.bias;
        srv.request.controller_parameters.mrft_no_switch_delay = _update_msg->param.no_switch_delay_in_ms;
        srv.request.controller_parameters.mrft_conf_samples = _update_msg->param.num_of_peak_conf_samples;
        bool success1 = false;
        bool success2 = false;
        bool success3 = false; 
        bool success4 = false;
        if( (int)_update_msg->param.id <= (int)MRFT_ID::MRFT_Z){
            success1 = m_client_mrft_x.call(srv);
            success2 = m_client_mrft_y.call(srv);
            success3 = m_client_mrft_z.call(srv);
            success4 = true;
        }else{
            success1 = m_client_mrft_roll.call(srv);
            success2 = m_client_mrft_pitch.call(srv);
            success3 = m_client_mrft_yaw.call(srv);
            success4 = m_client_mrft_yaw_rate.call(srv);
        }
        if (success1 && success2 && success3 && success4) {
            ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
        }
        else {
            ROS_ERROR("Failed to call service /update_controller");
        }
    }
}

}