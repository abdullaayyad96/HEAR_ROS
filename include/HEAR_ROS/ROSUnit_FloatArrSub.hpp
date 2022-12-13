#ifndef ROSUNIT_FLOATARRSUB_HPP
#define ROSUNIT_FLOATARRSUB_HPP

#include "HEAR_ROS/ROSUnit_Sub.hpp"
#include "std_msgs/Float32MultiArray.h"

#include <vector>

namespace HEAR{

class ROSUnitFloatArrSub : public ROSUnit_Sub{
private:
    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        ((OutputPort<std::vector<float>>*)_output_port)->write(msg->data);
    }
public:
    ROSUnitFloatArrSub(ros::NodeHandle& nh, const std::string& topic_name, int idx){
        sub_ = nh.subscribe<std_msgs::Float32MultiArray>(topic_name, 10, &ROSUnitFloatArrSub::callback, this);
        _output_port = new OutputPort<std::vector<float>>(idx, 0);
        
        ((OutputPort<std::vector<float>>*)_output_port)->write({0,0,0,0,0,0});
        id_ = idx;
    }

    TYPE getType(){ return TYPE::FloatVec;}

};

}

#endif