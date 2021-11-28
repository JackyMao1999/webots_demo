/************************************************* 
Copyright:Webots Demo
Author: 锡城筱凯
Date:2021-06-30 
Blog：https://blog.csdn.net/xiaokai1999
Change: 2021-11-20
Description:Webots Demo 通过手柄控制机器人移动
**************************************************/  
#include <signal.h>
#include <locale.h> 
#include <webots_ros.h> 
#include <sensor_msgs/Joy.h>

ros::NodeHandle *n;

const int TIME_STEP = 32;                   // 时钟
const int NMOTORS = 2;                      // 电机数量
const float MAX_SPEED = 2.0;                // 电机最大速度
const std::string ROBOT_NAME = "robot/";    // ROBOT名称 
double speeds[NMOTORS]={0.0,0.0};           // 电机速度值 0.0～10.0

static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"};// 控制位置电机名称

static int controllerCount;
static std::vector<std::string> controllerList; 

Webots w = Webots(TIME_STEP,ROBOT_NAME);
/*******************************************************
* Function name ：updateSpeed
* Description   ：将速度请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updateSpeed() {   
    for (int i = 0; i < NMOTORS; ++i) {
        // 更新速度
        w.SetMotorsVelocity(n, motorNames[i], -speeds[i]);
    }
}

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    //将控制器名加入到列表中
    controllerList.push_back(name->data);
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
    w.Quit(n);
}

/*******************************************************
* Function name ：手柄返回函数
* Description   ：当手柄动作，就会进入此函数内
* Parameter     ：
        @value   返回的值
* Return        ：无
**********************************************************/
void JoyDataCallback(const sensor_msgs::Joy::ConstPtr &value){
    // 发送控制变量
    int send = 0;
    if (value->buttons[2]){
        // 左转
        speeds[0] = -5.0;
        speeds[1] = 5.0;
        send=1;
    }
    else if(value->buttons[3]){
        // 前进
        speeds[0] = 5.0;
        speeds[1] = 5.0;
        send=1;
    }
    else if(value->buttons[0]){
        // 后退
        speeds[0] = -5.0;
        speeds[1] = -5.0;
        send=1;
    }
    else if(value->buttons[1]){
        //  右转
        speeds[0] = 5.0;
        speeds[1] = -5.0;
        send=1;
    }
    //当接收到信息时才会更新速度值
    if (send){
        updateSpeed();
        send=0;
    } 
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "zh_CN.utf8"); // 用于显示中文字符
    std::string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    w.Init(n, nameSub, controllerCount, controllerList);
    w.InitMotors(n, motorNames, NMOTORS);

    ros::Subscriber joySub;
    joySub = n->subscribe("/joy",1,JoyDataCallback);// 订阅手柄发来的topic话题
    while (joySub.getNumPublishers() == 0) {}
    while (ros::ok()) {   
        ros::spinOnce();
    } 
    w.Quit(n);
    return 0;
}