#include "HEAR_ROS/ROSUnit_UpdateMRFTContClnt.hpp"

namespace HEAR{

ROSUnit_UpdateMRFTContClnt::ROSUnit_UpdateMRFTContClnt(ros::NodeHandle& t_main_handler, std::string Drone_Name, std::string Channel) : nh_(t_main_handler){

    m_client_mrft = nh_.serviceClient<hear_msgs::Update_Controller_MRFT>(Drone_Name + "update_controller/mrft/" + Channel);

}

ROSUnit_UpdateMRFTContClnt::~ROSUnit_UpdateMRFTContClnt() {

}

bool ROSUnit_UpdateMRFTContClnt::process(BaseMsg* t_msg) {
    MRFT_UpdateMsg* _update_msg = (MRFT_UpdateMsg*)t_msg;
    hear_msgs::Update_Controller_MRFT srv;
    srv.request.controller_parameters.id = (int)_update_msg->param.id;
    srv.request.controller_parameters.mrft_beta = _update_msg->param.beta;
    srv.request.controller_parameters.mrft_relay_amp = _update_msg->param.relay_amp;
    srv.request.controller_parameters.mrft_no_switch_delay = _update_msg->param.no_switch_delay_in_ms;
    srv.request.controller_parameters.mrft_conf_samples = _update_msg->param.num_of_peak_conf_samples;
    bool success = false;
    
    success = m_client_mrft.call(srv);
    
    if (success) {
        ROS_INFO("CONTROLLER UPDATED. id: %d", static_cast<int>(srv.request.controller_parameters.id));
        return true;
    }
    else {
        ROS_ERROR("Failed to call service /update_controller/mrft");
        return false;
    }
 
}

}
