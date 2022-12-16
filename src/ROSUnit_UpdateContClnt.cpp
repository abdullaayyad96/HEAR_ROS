#include "HEAR_ROS/ROSUnit_UpdateContClnt.hpp"

namespace HEAR{

ROSUnit_UpdateContClnt::ROSUnit_UpdateContClnt(ros::NodeHandle& t_main_handler, std::string Drone_Name) : nh_(t_main_handler){

    // PID of the outer loop
    m_client_pid_x = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/x");
    m_client_pid_y = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/y");
    m_client_pid_z = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/z");

    // PID of the inner loop
    m_client_pid_roll = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/roll");
    m_client_pid_pitch = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/pitch");
    m_client_pid_yaw = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/yaw");
    m_client_pid_yaw_rate = nh_.serviceClient<hear_msgs::Update_Controller_PID>(Drone_Name + "update_controller/pid/yaw_rate");

    // MRFT of the outer loop
    m_client_mrft_x = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/x");
    m_client_mrft_y = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/y");
    m_client_mrft_z = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/z");

    // MRFT of the inner loop
    m_client_mrft_roll = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/roll");
    m_client_mrft_pitch = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/pitch");
    m_client_mrft_yaw = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/yaw");
    m_client_mrft_yaw_rate = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/yaw_rate");

    // Bounding of the outer loop
    m_client_bounding_x = nh_.serviceClient<hear_msgs::Update_Controller_Bounding>(Drone_Name + "update_controller/bounding/x");
    m_client_bounding_y = nh_.serviceClient<hear_msgs::Update_Controller_Bounding>(Drone_Name + "update_controller/bounding/y");
    m_client_bounding_z = nh_.serviceClient<hear_msgs::Update_Controller_Bounding>(Drone_Name + "update_controller/bounding/z");
}

ROSUnit_UpdateContClnt::~ROSUnit_UpdateContClnt() {

}

bool ROSUnit_UpdateContClnt::process(UpdateMsg* t_msg, BLOCK_ID ID) {
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
        bool success = false;
        
        switch ((int)_update_msg->param.id)
        {
            case (int)PID_ID::PID_X:
            {
                success = m_client_pid_x.call(srv);
            }
            break;
            case (int)PID_ID::PID_Y:
            {
                success = m_client_pid_y.call(srv);
            }
            break;
            case (int)PID_ID::PID_Z:
            {
                success = m_client_pid_z.call(srv);
            }
            break;
            case (int)PID_ID::PID_ROLL:
            {
                success = m_client_pid_roll.call(srv);
            }
            break;
            case (int)PID_ID::PID_PITCH:
            {
                success = m_client_pid_pitch.call(srv);
            }
            break;
            case (int)PID_ID::PID_YAW:
            {
                success = m_client_pid_yaw.call(srv);
            }
            break;
            case (int)PID_ID::PID_YAW_RATE:
            {
                success = m_client_pid_yaw_rate.call(srv);
            }
            break;
            default:
            {
                std::cerr <<"Invalid Controller ID type" <<std::endl;
                assert(false);
                break;
            }
        }
        if (success) {
            ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
            return true;
        }
        else {
            ROS_ERROR("Failed to call service /update_controller/pid");
            return false;
        }
    }
    else if(ID == MRFT) {
        MRFT_UpdateMsg* _update_msg = (MRFT_UpdateMsg*)t_msg;
        hear_msgs::Update_Controller_MRFT srv;
        srv.request.controller_parameters.id = static_cast<int>(_update_msg->param.id);
        srv.request.controller_parameters.mrft_beta = _update_msg->param.beta;
        srv.request.controller_parameters.mrft_relay_amp = _update_msg->param.relay_amp;
        srv.request.controller_parameters.mrft_no_switch_delay = _update_msg->param.no_switch_delay_in_ms;
        srv.request.controller_parameters.mrft_conf_samples = _update_msg->param.num_of_peak_conf_samples;
        bool success = false;
        
        switch((int)_update_msg->param.id)
        {
            case (int)MRFT_ID::MRFT_X:
            {
                success = m_client_mrft_x.call(srv);
            }
            break;
            case (int)MRFT_ID::MRFT_Y:
            {
                success = m_client_mrft_y.call(srv);
            }
            break;
            case (int)MRFT_ID::MRFT_Z:
            {
                success = m_client_mrft_z.call(srv);
            }
            break;
            case (int)MRFT_ID::MRFT_ROLL:
            {
                success = m_client_mrft_roll.call(srv);
            }
            break;
            case (int)MRFT_ID::MRFT_PITCH:
            {
                success = m_client_mrft_pitch.call(srv);
            }
            break;
            case (int)MRFT_ID::MRFT_YAW:
            {
                success = m_client_mrft_yaw.call(srv);
            }
            break;
            case (int)MRFT_ID::MRFT_YAW_RATE:
            {
                success = m_client_mrft_yaw_rate.call(srv);
            }
            break;
            default:
            {
                std::cerr <<"Invalid Controller ID type" <<std::endl;
                assert(false);
                break;
            }

        }
        if (success) {
            ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
            return true;
        }
        else {
            ROS_ERROR("Failed to call service /update_controller/mrft");
            return false;
        }
    }
    else if(ID == BOUNDINGCTRL){
        BoundingCtrl_UpdateMsg* _update_msg = (BoundingCtrl_UpdateMsg*)t_msg;
        hear_msgs::Update_Controller_Bounding srv;
        srv.request.controller_parameters.id = static_cast<int>(_update_msg->param.id);
        srv.request.controller_parameters.bounding_eps_1 = _update_msg->param.eps_1;
        srv.request.controller_parameters.bounding_eps_2 = _update_msg->param.eps_2;
        srv.request.controller_parameters.bounding_h_o1 = _update_msg->param.h_o1;
        srv.request.controller_parameters.bounding_h_o2 = _update_msg->param.h_o2;
        bool success = false;
        
        switch((int)_update_msg->param.id)
        {
            case (int)BOUNDING_ID::BOUNDING_X:
            {
                success = m_client_bounding_x.call(srv);
            }
            break;
            case (int)BOUNDING_ID::BOUNDING_Y:
            {
                success = m_client_bounding_y.call(srv);
            }
            break;
            case (int)BOUNDING_ID::BOUNDING_Z:
            {
                success = m_client_bounding_z.call(srv);
            }
            break;
            default:
            {
                std::cerr <<"Invalid Controller ID type" <<std::endl;
                assert(false);
                break;
            }

        }
        if (success) {
            ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
            return true;
        }
        else {
            ROS_ERROR("Failed to call service /update_controller/bounding");
            return false;
        }
    }
    else{
        ROS_ERROR("Failed to update controller");
        return false;
    }
}

}
