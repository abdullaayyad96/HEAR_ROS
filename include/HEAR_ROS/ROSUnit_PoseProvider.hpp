#ifndef ROSUNIT_POSEPROVIDER_HPP
#define ROSUNIT_POSEPROVIDER_HPP

#include <vector>

#include "ros/ros.h"
#include "/home/orbit3/HEAR/devel/include/mavros_msgs/VehicleAngularVelocity.h"
// #include <mavros_msgs/VehicleAngularVelocity.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <hear_msgs/set_float.h>

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{

class ROSUnit_PoseProvider{
private:
    ros::NodeHandle nh_;
    ros::Subscriber opti_sub, xsens_ori_sub, xsens_ang_vel_sub, xsens_free_acc_sub, px4_ang_vel_sub;
    ros::ServiceServer m_server;
    
    ExternalOutputPort<Vector3D<float>>* opti_pos_port;
    ExternalOutputPort<Vector3D<float>>* opti_vel_port;
    ExternalOutputPort<Vector3D<float>>* opti_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_acc_port;
    ExternalOutputPort<Vector3D<float>>* imu_angular_rt_port;
    ExternalOutputPort<Vector3D<float>>* px4_imu_angular_rt_port;
    void callback_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& );
    void callback_px4_pose(const geometry_msgs::PoseStamped::ConstPtr& );
    void callback_ori(const geometry_msgs::QuaternionStamped::ConstPtr& );
    void callback_free_acc(const geometry_msgs::Vector3Stamped::ConstPtr& );
    void callback_angular_vel(const geometry_msgs::Vector3Stamped::ConstPtr&);
    void callback_px4_angular_vel(const mavros_msgs::VehicleAngularVelocity::ConstPtr&);
    bool srv_callback(hear_msgs::set_float::Request&, hear_msgs::set_float::Response&);
    tf2::Matrix3x3 rot_offset;
    tf2::Vector3 trans_offset;

    tf2::Vector3 opti_pos, prev_pos, opti_vel, prev_diff, _hold;
    ros::Time prevT;
    uint8_t first_read = 0;
    const float PEAK_THRESH = 0.35;

public:
    void process(){}
    ROSUnit_PoseProvider(ros::NodeHandle& nh);
    ~ROSUnit_PoseProvider(){}
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerOptiPose(std::string t_name);
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerPX4Pose(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuOri(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAngularRate(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerPX4ImuAngularRate(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAcceleration(std::string t_name);
};

}

#endif