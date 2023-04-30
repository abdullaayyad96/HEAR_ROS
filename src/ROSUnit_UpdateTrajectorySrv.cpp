#include "HEAR_ROS/ROSUnit_UpdateTrajectorySrv.hpp"

namespace HEAR{

ExternalTrigger<BaseMsg>* ROSUnit_UpdateTrajectorySrv::registerServer(const std::string &service_topic){
    ext_trig = new ExternalTrigger<BaseMsg>;
    this->m_server = this->nh_.advertiseService(service_topic, &ROSUnit_UpdateTrajectorySrv::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_UpdateTrajectorySrv::srv_callback(hear_msgs::Update_Trajectory::Request& req, hear_msgs::Update_Trajectory::Response& res){
    Trajectory_UpdateMsg msg;
    msg.param._trajectoryType = (int)req.trajectory_parameters.trajectoryType;
    msg.param._samplingType = (int)req.trajectory_parameters.samplingType;
    msg.param._transformationType = (int)req.trajectory_parameters.transformationType;
    msg.param.rot = req.trajectory_parameters.rot;
    msg.param.trans = req.trajectory_parameters.trans;
    msg.param.scale = req.trajectory_parameters.scale;
    msg.param.NumSamples = req.trajectory_parameters.NumSamples;
    msg.param.ClearQ = req.trajectory_parameters.ClearQ;
    
    ext_trig->UpdateCallback((BaseMsg*)&msg);
    return true;
}

}