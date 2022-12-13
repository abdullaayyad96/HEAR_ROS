#ifndef ROSUNIT_MULTIPORTSWITCHSRV_HPP
#define ROSUNIT_MULTIPORTSWITCHSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_float.h>

#include "HEAR_core/ExternalTrigger.hpp"

#include <string>
#include <vector>
#include <utility>


namespace HEAR{

class ROSUnit_MultiportSwitchSrv {
private:
    UpdateTrigger* _pid_trig;
    UpdateTrigger* _mrft_trig;
    UpdateTrigger* _bounding_trig;
    std::vector <std::pair<UpdateTrigger*, bool>> _switch_trig;
    ros::ServiceServer m_server;
    bool srvCallback(hear_msgs::set_float::Request& , hear_msgs::set_float::Response& );
public:
    ROSUnit_MultiportSwitchSrv(ros::NodeHandle& nh , const std::string& topic_name ){
        m_server = nh.advertiseService(topic_name, &ROSUnit_MultiportSwitchSrv::srvCallback, this);
        _pid_trig = new UpdateTrigger();
        _mrft_trig = new UpdateTrigger();
        _bounding_trig = new UpdateTrigger();
    }
    ExternalTrigger* getMRFTTrig(){ return _mrft_trig;}
    ExternalTrigger* getPIDTrig(){ return _pid_trig;}
    ExternalTrigger* getBoundingTrig(){ return _bounding_trig;}
    ExternalTrigger* registerSwitchTrig(bool inverted_logic = false){
        auto trig = new UpdateTrigger();
        _switch_trig.push_back(std::make_pair(trig, inverted_logic));
        return trig;
    }

};

bool ROSUnit_MultiportSwitchSrv::srvCallback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res){
    float data = req.data;
    if(data == 1){
        BoolMsg msg;
        msg.data = false;
        _pid_trig->UpdateCallback((UpdateMsg*)&msg);
        _bounding_trig->UpdateCallback((UpdateMsg*)&msg);
        msg.data = true;
        _mrft_trig->UpdateCallback((UpdateMsg*)&msg);
        MultiportSwitchMsg sw_msg;
        for (const auto& trig : _switch_trig){
            if(trig.second){
                sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON3;
            }
            else{
                sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON1;
            }
            trig.first->UpdateCallback((UpdateMsg*)&sw_msg);
        }

    }
    else if (data == 2){
        BoolMsg msg;
        msg.data = false;
        _mrft_trig->UpdateCallback((UpdateMsg*)&msg);
        _pid_trig->UpdateCallback((UpdateMsg*)&msg);
        msg.data = true;
        _bounding_trig->UpdateCallback((UpdateMsg*)&msg);
        MultiportSwitchMsg sw_msg;
        for (const auto& trig : _switch_trig){
            if(trig.second){
                sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON3;
            }
            else{
                sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON2;
            }
            trig.first->UpdateCallback((UpdateMsg*)&sw_msg);
        }
    }
    else{
        BoolMsg msg;
        msg.data = false;
        _mrft_trig->UpdateCallback((UpdateMsg*)&msg);
        _bounding_trig->UpdateCallback((UpdateMsg*)&msg);
        msg.data = true;
        _pid_trig->UpdateCallback((UpdateMsg*)&msg);
        MultiportSwitchMsg sw_msg;
        for (const auto& trig : _switch_trig){
            if(trig.second){
                sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON1;
            }
            else{
                sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON3;
            }
            trig.first->UpdateCallback((UpdateMsg*)&sw_msg);
        }
    }
    return true;
}

}

#endif
