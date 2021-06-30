/************************************************* 
Copyright:Webots Demo
Author: 锡城筱凯
Date:2021-06-30 
Blog：https://blog.csdn.net/xiaokai1999
Description:Webots Demo 机器人导航底层控制代码
**************************************************/  
#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h> 
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
using namespace std;

#define TIME_STEP 32                        // 时钟
#define NMOTORS 2                           // 电机数量
#define MAX_SPEED 2.0                       // 电机最大速度
#define ROBOT_NAME "robot/"                 // ROBOT名称

ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::ServiceClient timeStepClient;          // 时钟通讯service客户端
webots_ros::set_int timeStepSrv;            // 时钟服务数据

ros::ServiceClient set_velocity_client;     // 电机速度通讯service客户端
webots_ros::set_float set_velocity_srv;     // 电机速度服务数据

ros::ServiceClient set_position_client;     // 电机位置通讯service客户端
webots_ros::set_float set_position_srv;     // 电机位置服务数据

double speeds[NMOTORS]={0.0,0.0};           // 四电机速度值 0～10
static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"};// 控制位置电机名称

/*******************************************************
* Function name ：updateSpeed
* Description   ：将速度请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updateSpeed() {   
    for (int i = 0; i < NMOTORS; ++i) {
        // 更新速度
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string(ROBOT_NAME) + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = -speeds[i];
        set_velocity_client.call(set_velocity_srv);
    }
}

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   退出信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/robot' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

/*******************************************************
* Function name ：cmdvelDataCallback
* Description   ：cmd_vel topic回调函数
* Parameter     ：
        @value   返回的值
* Return        ：无
**********************************************************/
void cmdvelDataCallback(const geometry_msgs::Twist::ConstPtr &value){
    float linear_temp=0, angular_temp=0;//暂存的线速度和角速度,
    float L = 0.6;//两轮之间的距离
    angular_temp = value->angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = value->linear.x ;//获取/cmd_vel的线速度.m/s
    // 将转换好的小车速度分量为左右轮速度
    // 根据双轮差动底盘算法计算
    // v(linear_temp)为底盘中心线速度；w(angular_temp)为底盘中心角速度
    // Vl(speeds[0]),Vr(speeds[1])为左右两轮的速度
    // d(float L = 0.06)为轮子离底盘中心的位置
    // v = (Vr+Vl)/2      w = (Vr-Vl)/2d
    speeds[0]  = 10.0*(2.0*linear_temp - L*angular_temp)/2.0;
    speeds[1]  = 10.0*(2.0*linear_temp + L*angular_temp)/2.0;
    updateSpeed();
    ROS_INFO("left_vel:%lf,  right_vel:%lf", speeds[0], speeds[1]);
}

int main(int argc, char **argv) {
    std::string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;

    signal(SIGINT, quit);
    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();
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

    // 初始化电机  
    for (int i = 0; i < NMOTORS; ++i) {
        // position速度控制时设置为缺省值INFINITY   
        // 初始化四个控制位置的电机
        set_position_client = n->serviceClient<webots_ros::set_float>(string(ROBOT_NAME) + string(motorNames[i]) + string("/set_position"));   
        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)     
            ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);   
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string(ROBOT_NAME) + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 0.0;   
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
    }   

    ros::Subscriber cmdvelSub;
    cmdvelSub = n->subscribe("/cmd_vel",1,cmdvelDataCallback);// 监听/cmd_vel，获取导航算法发过来的数据
    while (cmdvelSub.getNumPublishers() == 0) {}
    while (ros::ok()) {   
        ros::spinOnce();
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success){  
            ROS_ERROR("Failed to call service time_step for next step.");     
            break;   
        }   
        ros::spinOnce();
    } 
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown(); 
    return 0;
}
