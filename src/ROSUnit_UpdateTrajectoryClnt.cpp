#include "HEAR_ROS/ROSUnit_UpdateTrajectoryClnt.hpp"

namespace HEAR{

ROSUnit_UpdateTrajectoryClnt::ROSUnit_UpdateTrajectoryClnt(ros::NodeHandle& t_main_handler, std::string Drone_Name) : nh_(t_main_handler){

    m_client = nh_.serviceClient<hear_msgs::Update_Trajectory>(Drone_Name + "generate_trajectory");
    

}

ROSUnit_UpdateTrajectoryClnt::~ROSUnit_UpdateTrajectoryClnt() {

}

bool ROSUnit_UpdateTrajectoryClnt::process(BaseMsg* t_msg) {
    Trajectory_UpdateMsg* _update_msg = (Trajectory_UpdateMsg*)t_msg;
    hear_msgs::Update_Trajectory srv;
    srv.request.trajectory_parameters.trajectoryType = (int)_update_msg->param._trajectoryType;
    srv.request.trajectory_parameters.transformationType = (int)_update_msg->param._transformationType;
    srv.request.trajectory_parameters.scale = _update_msg->param.scale;
    srv.request.trajectory_parameters.rot = _update_msg->param.rot;
    srv.request.trajectory_parameters.trans = _update_msg->param.trans;
    srv.request.trajectory_parameters.NumSamples = _update_msg->param.NumSamples;
    srv.request.trajectory_parameters.ClearQ = _update_msg->param.ClearQ;
    bool success = false;
    success = m_client.call(srv);
    
    if (success) {
        ROS_INFO("TRAJECTORY UPDATED.");
        return true;
    }
    else {
        ROS_ERROR("Failed to call service /generate_trajectory");
        return false;
    }
    
}

}
