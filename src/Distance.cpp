#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>


turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
bool turtle1_updated = false;
bool turtle2_updated = false;


void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
    turtle1_updated = true;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
    turtle2_updated = true;
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "distance");
    ros::NodeHandle nh;
    
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
    
    // Publisher for the distance between turtles
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/turtles/distance", 10);
    
    // creating Publishers in order to control turtle velocities
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    
    ros::Rate rate(10);  // 10 Hz
    
    
   while (ros::ok()) {
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
    
    
