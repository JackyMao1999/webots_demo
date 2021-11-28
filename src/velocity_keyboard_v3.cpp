/************************************************* 
Copyright:Webots Demo
Author: 锡城筱凯
Date:2021-06-30 
Blog：https://blog.csdn.net/xiaokai1999
Change: 2021-11-07
Description:Webots Demo 通过webots控制机器人移动
**************************************************/  
#include <signal.h>
#include <locale.h> 
#include <webots_ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle *n;

const int TIME_STEP = 32;                   // 时钟
const int NMOTORS = 2;                      // 电机数量
const float MAX_SPEED = 2.0;                // 电机最大速度
const std::string ROBOT_NAME = "robot/";    // ROBOT名称 
double speeds[NMOTORS]={0.0,0.0};           // 电机速度值 0.0～10.0
float linear_temp=0, angular_temp=0;        // 暂存的线速度和角速度

static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"};// 控制位置电机名称

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::Publisher pub_speed;                   // 发布 /vel
Webots w = Webots(TIME_STEP,ROBOT_NAME);

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
        w.SetMotorsVelocity(n, motorNames[i], -speeds[i]);
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
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    //将控制器名加入到列表中
    controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
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
    switch (value->data){
        // 左转
        case 314:
            angular_temp-=0.1;
            break;
        // 前进
        case 315:
            linear_temp += 0.1;
            break;
        // 右转
        case 316:
            angular_temp+=0.1;
            break;
        // 后退
        case 317:
            linear_temp-=0.1;
            break;
        // 停止
        case 32:
            linear_temp = 0;
            angular_temp = 0;
            break;
        default:
            break;
    }
}
/*******************************************************
* Function name ：cmdvel返回函数
* Description   ：获取导航返回的角速度和线速度
* Parameter     ：
        @value   返回的值
* Return        ：无
**********************************************************/
void cmdvelDataCallback(const geometry_msgs::Twist::ConstPtr &value)
{
    
    angular_temp = value->angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = value->linear.x ;//获取/cmd_vel的线速度.m/s  
    
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
int main(int argc, char **argv) {
    //设置中文
    setlocale(LC_CTYPE,"zh_CN.utf8");
    std::string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 10, controllerNameCallback);
    w.Init(n, nameSub, controllerCount, controllerList);
    w.InitMotors(n, motorNames, NMOTORS);
    
    ros::Subscriber cmdvelSub;
    cmdvelSub = n->subscribe("/cmd_vel",1,cmdvelDataCallback);
    pub_speed = n->advertise<nav_msgs::Odometry>("/vel",1);
    if(!w.EnableService(n, "keyboard")){
        ros::Subscriber keyboardSub;
        keyboardSub = n->subscribe(std::string(ROBOT_NAME)+std::string("keyboard/key"),1,keyboardDataCallback);
        while (keyboardSub.getNumPublishers() == 0) {}
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
            if (w.ChecktimeStep())break;    
            ros::spinOnce();
        } 
    }
    w.Quit(n);
    return 0;
}