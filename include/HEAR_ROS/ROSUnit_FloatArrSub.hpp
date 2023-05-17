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
        sub_ = nh.subscribe<std_msgs::Float32MultiArray>(topic_name, 1, &ROSUnitFloatArrSub::callback, this);
        _output_port = new OutputPort<std::vector<float>>(idx, 0);
        std::vector<float> _commands {0,0,0,0,0,0};
        for (int i = 0; i < 6; i++){
            _commands[i] = 1000.0;
        }
        
        ((OutputPort<std::vector<float>>*)_output_port)->write(_commands);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::FloatVec;}

};

}

#endif