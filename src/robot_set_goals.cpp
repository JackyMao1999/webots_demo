#include "string.h"
#include "webots_demo/Goalname.h" 
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
 
using namespace std;
 
ros::NodeHandle *n;
ros::Publisher pub_goal;
void goalCallback(const webots_demo::Goalname::ConstPtr &value);

int main(int argc, char **argv) {
   
    // create a node named 'robot' on ROS network
    ros::init(argc, argv, "robot_set_goal");
    n = new ros::NodeHandle;
    ros::Subscriber sub_goal;
    sub_goal = n->subscribe("/robot/goal",1,goalCallback);
    
    pub_goal = n->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
    ROS_INFO("Started success ");
    ros::spin();
}
void goalCallback(const webots_demo::Goalname::ConstPtr &value){
    int isture=0;
    string goal_name = value->goal_name;
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.seq = 1;
    target_pose.header.frame_id = "map";
    
    if (goal_name == "bedroom")
    {
        target_pose.pose.position.x = -1.3;
        target_pose.pose.position.y = -2.5;
        target_pose.pose.orientation.z = 0.0016;
        target_pose.pose.orientation.w = -0.6538;
        isture = 1;
    }

    if (isture)
    {
        target_pose.header.stamp = ros::Time::now();
        pub_goal.publish(target_pose);
        ROS_INFO("Ready to go to the goal %s",goal_name.c_str());
    }
    else
    {
        ROS_ERROR("Can't compare the goal");
    }
}
