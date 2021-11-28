/************************************************* 
Copyright:Webots_ros API
Author: 锡城筱凯
Date:2021-11-07 
Blog：https://blog.csdn.net/xiaokai1999
Description:Webots_ros官方库的整合库，简便易用
**************************************************/  
#ifndef _WEBOTS_ROS_H
#define _WEBOTS_ROS_H


#include "ros/ros.h"
#include <std_msgs/String.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Int32Stamped.h>


class Webots
{
private:
    // 同步Webots所设置的时钟
    int TIME_STEP;
    // 机器人的名称，对应Webots中Robot下的name属性
    std::string ROBOT_NAME;
public:
    // 时钟通讯service客户端
    ros::ServiceClient timeStepClient;  
    // 时钟服务数据        
    webots_ros::set_int timeStepSrv;            
    // Webots类构造函数，设置必须变量
    Webots(int TIME_STEP,std::string ROBOT_NAME);
    // 初始化Webots和Ros之间的桥梁，成功返回0，失败返回1
    int Init(ros::NodeHandle *n, ros::Subscriber nameSub, const int &controllerCount, const std::vector<std::string> &controllerList);
    // 初始化电机，成功返回0，失败返回1
    int InitMotors(ros::NodeHandle *n, const char *motorNames[], int NMOTORS); 
    // 设置电机位置属性，成功返回0，失败返回1
    int SetMotorsPositon(ros::NodeHandle *n, const char *motorNames, float value);
    // 设置电机速度属性，成功返回0，失败返回1
    int SetMotorsVelocity(ros::NodeHandle *n, const char *motorNames, float value);
    // 使能Webots服务，成功返回0，失败返回1
    int EnableService(ros::NodeHandle *n, std::string Service_name);
    // 检测时钟通讯，成功返回0，失败返回1
    int ChecktimeStep();
    // 退出函数
    void Quit(ros::NodeHandle *n);
};
Webots::Webots(int TIME_STEP,std::string ROBOT_NAME){
    this->TIME_STEP = TIME_STEP;
    this->ROBOT_NAME = ROBOT_NAME;
}
int Webots::Init(ros::NodeHandle *n, ros::Subscriber nameSub, const int &controllerCount, const std::vector<std::string> &controllerList){
    std::string controllerName;
    // 订阅webots中所有可用的model_name
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    
    // 服务订阅time_step和webots保持同步
    timeStepClient = n->serviceClient<webots_ros::set_int>("robot/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // 如果在webots中有多个控制器的话，需要让用户选择一个控制器
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
            controllerName = controllerList[wantedController - 1];
        else {
            ROS_ERROR("Invalid number for controller choice.");
            return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // 退出主题，因为已经不重要了
    nameSub.shutdown();
}
int Webots::InitMotors(ros::NodeHandle *n, const char *motorNames[], int NMOTORS){
    for (int i = 0; i < NMOTORS; ++i) {
        SetMotorsPositon(n, motorNames[i], INFINITY);
        SetMotorsVelocity(n,motorNames[i], 0.0);
    }
}
int Webots::SetMotorsPositon(ros::NodeHandle *n, const char *motorNames, float value){
    ros::ServiceClient set_position_client;     // 电机位置通讯service客户端
    webots_ros::set_float set_position_srv;     // 电机位置服务数据
    set_position_client = n->serviceClient<webots_ros::set_float>(ROBOT_NAME + "/" + std::string(motorNames) + std::string("/set_position"));   
    set_position_srv.request.value = value;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success){
        ROS_INFO("Position set to %f for motor %s.", value, motorNames);   
        return 0;
    }   
    else{
        ROS_ERROR("Failed to call service set_position on motor %s.", motorNames);
        return 1;
    }
}
int Webots::SetMotorsVelocity(ros::NodeHandle *n, const char *motorNames, float value){
    ros::ServiceClient set_velocity_client;     // 电机速度通讯service客户端
    webots_ros::set_float set_velocity_srv;     // 电机速度服务数据 
    set_velocity_client = n->serviceClient<webots_ros::set_float>(ROBOT_NAME + "/" + std::string(motorNames) + std::string("/set_velocity"));   
    set_velocity_srv.request.value = value;   
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
        ROS_INFO("Velocity set to %f for motor %s.",value, motorNames);   
    else     
        ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames); 
}
int Webots::EnableService(ros::NodeHandle *n, std::string Service_name){
    ros::ServiceClient EnableClient;
    webots_ros::set_int Enablesrv;
   
    EnableClient = n->serviceClient<webots_ros::set_int>(ROBOT_NAME + "/" + Service_name + "/enable");
    Enablesrv.request.value = TIME_STEP;
    if (EnableClient.call(Enablesrv) && Enablesrv.response.success){
        ROS_INFO("Enable %s successful", Service_name.c_str());
        return 0;
    } 
    else{
        ROS_ERROR("Could not enable %s, success = %d.", Service_name.c_str(), Enablesrv.response.success);
        return 1;
    }
}


int Webots::ChecktimeStep(){
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success){  
        ROS_ERROR("Failed to call service time_step for next step.");     
        return 1;
    }else return 0;
}

void Webots::Quit(ros::NodeHandle *n){
    ROS_INFO("Stopped the node.");
    delete n;
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}
#endif