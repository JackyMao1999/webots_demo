/************************************************* 
Copyright:Webots Demo
Author: 锡城筱凯
Date:2021-06-30 
Blog：https://blog.csdn.net/xiaokai1999
Change: 2021-11-20
Description:Webots Demo 机器人导航底层控制代码
**************************************************/  
#include <signal.h>
#include <geometry_msgs/Twist.h> 
#include <webots_ros.h> 

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
    w.Quit(n);
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
    w.Init(n, nameSub, controllerCount, controllerList);
    w.InitMotors(n, motorNames, NMOTORS);

    ros::Subscriber cmdvelSub;
    cmdvelSub = n->subscribe("/cmd_vel",1,cmdvelDataCallback);// 监听/cmd_vel，获取导航算法发过来的数据
    while (cmdvelSub.getNumPublishers() == 0) {}
    while (ros::ok()) {   
        if (w.ChecktimeStep())break;
        ros::spinOnce();
    } 
    w.Quit(n);
    return 0;
}
