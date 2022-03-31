#include <iostream>

#include <ros.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <vector>
#include "SP_node.h"

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
ros::Publisher statePub("/current_state", &odom_msg);


traj_type* next_traj = new traj_type; 
pose* curr_pose = new pose; 

void trajCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint &msg){
    printf("Received Subscribed Message");

    next_traj->pos[0] = msg.transforms[0].translation.x; 
    next_traj->pos[1] = msg.transforms[0].translation.y; 
    next_traj->pos[2] = msg.transforms[0].translation.z; 

    next_traj->vel[0] = 0.0; 
    next_traj->vel[1] = 0.0; 
    next_traj->vel[2] = 0.0;

    // x y z w order
    next_traj->quat[0] = msg.transforms[0].rotation.x; 
    next_traj->quat[1] = msg.transforms[0].rotation.y; 
    next_traj->quat[2] = msg.transforms[0].rotation.z;  
    next_traj->quat[3] = msg.transforms[0].rotation.w;  

    next_traj->rates[0] = 0.0; 
    next_traj->rates[1] = 0.0; 
    next_traj->rates[2] = 0.0;

}

void poseCallback(const geometry_msgs::Pose &msg){

    curr_pose->pos[0] = msg.position.x; 
    curr_pose->pos[1] = msg.position.y; 
    curr_pose->pos[2] = msg.position.z; 

    // x y z w order
    curr_pose->quat[0] = msg.orientation.x; 
    curr_pose->quat[1] = msg.orientation.y; 
    curr_pose->quat[2] = msg.orientation.z;  
    curr_pose->quat[3] = msg.orientation.w;  

}

ros::Subscriber<trajectory_msgs::MultiDOFJointTrajectoryPoint> sub("/desired_state", trajCallback);
ros::Subscriber<geometry_msgs::Pose> pose_sub("/position_estimate", poseCallback); 

char *IP = "192.168.6.1";

traj_type send_traj(void){

    return *next_traj; 
}



int main()
{
    nh.initNode(IP);
    nh.subscribe(sub);
    nh.advertise(statePub);
    while(1){
        // double *pos = get_pos_c();   
        
        // <x, y, z, vx, vy, vz, qw, qx, qy, qz, vr, vp, vy>
        // odom_msg.pose.pose.position.x = pos[0]; 
        // odom_msg.pose.pose.position.y = pos[1]; 
        // odom_msg.pose.pose.position.z = pos[2];
        // odom_msg.twist.twist.linear.x = pos[3];
        // odom_msg.twist.twist.linear.y = pos[4];
        // odom_msg.twist.twist.linear.z = pos[5];
        // odom_msg.pose.pose.orientation.w = pos[6];
        // odom_msg.pose.pose.orientation.x = pos[7];
        // odom_msg.pose.pose.orientation.y = pos[8];
        // odom_msg.pose.pose.orientation.z = pos[9];
        // odom_msg.twist.twist.angular.x = pos[10];
        // odom_msg.twist.twist.angular.y = pos[11];
        // odom_msg.twist.twist.angular.z = pos[12];

       
        //state.data = //something receive from the rc_pilot
        statePub.publish(&odom_msg);
        nh.spinOnce();  
    }
}   



//ouble quatx, quaty, quatz, quatw
//quatx = odom_msg.pose.pose.orientation.x;
//quaty = odom_msg.pose.pose.orientation.y;
//quatz = odom_msg.pose.pose.orientation.z;
//quatw = odom_msg.pose.pose.orientation.w;
//tf::Quaternion q(quatx, quaty, quatz, quatw);
//tf::Matrix3x3 m(q);
//double roll, pitch, yaw;
//m.getRPY(roll, pitch, yaw);
//roll = pos[6];
//pitch = pos[7];
//yaw = pos[8];


