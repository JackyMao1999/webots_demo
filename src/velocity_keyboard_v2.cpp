#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include <locale.h> 
#include <nav_msgs/Odometry.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
 
using namespace std;

#define TIME_STEP 32    //时钟
#define NMOTORS 2       //电机数量
#define MAX_SPEED 2.0   //电机最大速度

ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::ServiceClient timeStepClient;          //时钟通讯客户端
webots_ros::set_int timeStepSrv;            //时钟服务数据

ros::ServiceClient set_velocity_client;
webots_ros::set_float set_velocity_srv;

ros::ServiceClient set_position_client;   
webots_ros::set_float set_position_srv;   

ros::Publisher pub_speed;

double speeds[NMOTORS]={0.0,0.0};// 四电机速度值 0～100
float linear_temp=0, angular_temp=0;    //暂存的线速度和角速度
// 匹配电机名
static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"};

/*******************************************************
* Function name ：updateSpeed
* Description   ：将速度请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updateSpeed() {   
    nav_msgs::Odometry speed_data;
    //两轮之间的距离
    float L = 0.6;
    speeds[0]  = 10.0*(2.0*linear_temp - L*angular_temp)/2.0;
    speeds[1]  = 10.0*(2.0*linear_temp + L*angular_temp)/2.0;
    for (int i = 0; i < NMOTORS; ++i) {
        // 更新速度
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/robot/") + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = -speeds[i];
        set_velocity_client.call(set_velocity_srv);
    }
    speed_data.header.stamp = ros::Time::now();
    speed_data.twist.twist.linear.x = linear_temp;
    speed_data.twist.twist.angular.z = angular_temp;
    pub_speed.publish(speed_data);
    speeds[0]=0;
    speeds[1]=0;
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
        @sig   信号
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
* Function name ：键盘返回函数
* Description   ：当键盘动作，就会进入此函数内
* Parameter     ：
        @value   返回的值
* Return        ：无
**********************************************************/
void keyboardDataCallback(const webots_ros::Int32Stamped::ConstPtr &value)
{
    switch (value->data)
    {
        case 314:
            angular_temp-=0.1;
            break;
        case 315:
            linear_temp += 0.1;
            break;
        case 316:
            angular_temp+=0.1;
            break;
        case 317:
            linear_temp-=0.1;
            break;
        case 32:
            linear_temp = 0;
            angular_temp = 0;
            break;
        default:
            break;
    }
}


int main(int argc, char **argv) {
   
    std::string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
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

    //初始化电机
    for (int i = 0; i < NMOTORS; ++i) {
        // position速度控制时设置为缺省值INFINITY   
        set_position_client = n->serviceClient<webots_ros::set_float>(string("/robot/") + string(motorNames[i]) + string("/set_position"));   
        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)     
            ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);
        // velocity初始速度设置为0   
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/robot/") + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 0.0;   
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
    }   
    
    pub_speed = n->advertise<nav_msgs::Odometry>("/vel",10);
    // 服务订阅键盘
    ros::ServiceClient keyboardEnableClient;
    webots_ros::set_int keyboardEnablesrv;
   
    keyboardEnableClient = n->serviceClient<webots_ros::set_int>("/robot/keyboard/enable");
    keyboardEnablesrv.request.value = TIME_STEP;
    if (keyboardEnableClient.call(keyboardEnablesrv) && keyboardEnablesrv.response.success)
    {
        ros::Subscriber keyboardSub;
        keyboardSub = n->subscribe("/robot/keyboard/key",1,keyboardDataCallback);
        while (keyboardSub.getNumPublishers() == 0) {}
        setlocale(LC_CTYPE,"zh_CN.utf8");//设置中文
        ROS_INFO("Keyboard enabled.");
        ROS_INFO("控制方向：");
        ROS_INFO("  ↑  ");
        ROS_INFO("← ↓ →");
        ROS_INFO("刹车：空格键");
        ROS_INFO("Use the arrows in Webots window to move the robot.");
        ROS_INFO("Press the End key to stop the node.");
        while (ros::ok()) {   
            ros::spinOnce();
            updateSpeed();
            if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
            {  
                ROS_ERROR("Failed to call service time_step for next step.");     
                break;   
            }   
            ros::spinOnce();
        } 
    }
    else
    ROS_ERROR("Could not enable keyboard, success = %d.", keyboardEnablesrv.response.success);

    
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown(); 
    return 0;

}
