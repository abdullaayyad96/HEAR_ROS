// #ifndef ROSUNIT_MULTIPORTSWITCHSRV_HPP
// #define ROSUNIT_MULTIPORTSWITCHSRV_HPP

// #include <ros/ros.h>
// #include <hear_msgs/set_float.h>

// #include "HEAR_core/ExternalTrigger.hpp"

// #include <string>
// #include <vector>
// #include <utility>


// namespace HEAR{

// class ROSUnit_MultiportSwitchSrv {
// private:
//     ExternalTrigger<BaseMsg>* _pid_trig;
//     ExternalTrigger<BaseMsg>* _mrft_trig;
//     ExternalTrigger<BaseMsg>* _bounding_trig;
//     std::vector <std::pair<ExternalTrigger<BaseMsg>*, bool>> _switch_trig;
//     ros::ServiceServer m_server;
//     bool srvCallback(hear_msgs::set_float::Request& , hear_msgs::set_float::Response& );
// public:
//     ROSUnit_MultiportSwitchSrv(ros::NodeHandle& nh , const std::string& topic_name ){
//         m_server = nh.advertiseService(topic_name, &ROSUnit_MultiportSwitchSrv::srvCallback, this);
//         _pid_trig = new ExternalTrigger<BaseMsg>();
//         _mrft_trig = new ExternalTrigger<BaseMsg>();
//         _bounding_trig = new ExternalTrigger<BaseMsg>();
//     }
//     ExternalTrigger<BaseMsg>* getMRFTTrig(){ return _mrft_trig;}
//     ExternalTrigger<BaseMsg>* getPIDTrig(){ return _pid_trig;}
//     ExternalTrigger<BaseMsg>* getBoundingTrig(){ return _bounding_trig;}
//     ExternalTrigger<BaseMsg>* registerSwitchTrig(bool inverted_logic = false){
//         auto trig = new ExternalTrigger<BaseMsg>();
//         _switch_trig.push_back(std::make_pair(trig, inverted_logic));
//         return trig;
//     }

// };

// bool ROSUnit_MultiportSwitchSrv::srvCallback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res){
//     float data = req.data;
//     if(data == 1){
//         BoolMsg msg;
//         msg.data = false;
//         _pid_trig->UpdateCallback((BaseMsg*)&msg);
//         _bounding_trig->UpdateCallback((BaseMsg*)&msg);
//         msg.data = true;
//         _mrft_trig->UpdateCallback((BaseMsg*)&msg);
//         MultiportSwitchMsg sw_msg;
//         for (const auto& trig : _switch_trig){
//             if(trig.second){
//                 sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON3;
//             }
//             else{
//                 sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON1;
//             }
//             trig.first->UpdateCallback((BaseMsg*)&sw_msg);
//         }

//     }
//     else if (data == 2){
//         BoolMsg msg;
//         msg.data = false;
//         _mrft_trig->UpdateCallback((BaseMsg*)&msg);
//         _pid_trig->UpdateCallback((BaseMsg*)&msg);
//         msg.data = true;
//         _bounding_trig->UpdateCallback((BaseMsg*)&msg);
//         MultiportSwitchMsg sw_msg;
//         for (const auto& trig : _switch_trig){
//             if(trig.second){
//                 sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON3;
//             }
//             else{
//                 sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON2;
//             }
//             trig.first->UpdateCallback((BaseMsg*)&sw_msg);
//         }
//     }
//     else{
//         BoolMsg msg;
//         msg.data = false;
//         _mrft_trig->UpdateCallback((BaseMsg*)&msg);
//         _bounding_trig->UpdateCallback((BaseMsg*)&msg);
//         msg.data = true;
//         _pid_trig->UpdateCallback((BaseMsg*)&msg);
//         MultiportSwitchMsg sw_msg;
//         for (const auto& trig : _switch_trig){
//             if(trig.second){
//                 sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON1;
//             }
//             else{
//                 sw_msg.sw_state = MULTIPORT_SWITCH_STATE::ON3;
//             }
//             trig.first->UpdateCallback((BaseMsg*)&sw_msg);
//         }
//     }
//     return true;
// }

// }

// #endif
