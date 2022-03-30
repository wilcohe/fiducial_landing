#include <ros.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <vector>
#include "SP_node.h"

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
ros::Publisher statePub("/current_state", &odom_msg);

traj_type* next_traj; 

void messageCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint &message){
    printf("Received Subscribed Message");



}
ros::Subscriber<trajectory_msgs::MultiDOFJointTrajectoryPoint> sub("/desired_state", messageCb);

char *IP = "192.168.6.1";

double* get_pos_c(){

    double arr[13]; 

    return arr; 

}

double* send_traj(void){

    double traj[13];
    traj_type next = *next_traj; 

    for (int i = 0; i < 3; i++){
        traj[i] = next.pos[i]; 
        traj[i + 3] = next.vel[i]; 
        traj[i + 10] = next.rates[i]; 
    }

    for (int i = 0; i < 4; i++)
        traj[i + 6] = next.quat[i]; 

    return traj; 
}



int main()
{
    // nh.initNode(IP);
    // nh.subscribe(sub);
    // nh.advertise(statePub);
    // while(1){
    //     double* pos = get_pos_c(); 
        
    //     // <x, y, z, vx, vy, vz, qw, qx, qy, qz, vr, vp, vy>
    //     odom_msg.pose.pose.position.x = pos[0]; 
    //     odom_msg.pose.pose.position.y = pos[1]; 
    //     odom_msg.pose.pose.position.z = pos[2];
    //     odom_msg.twist.twist.linear.x = pos[3];
    //     odom_msg.twist.twist.linear.y = pos[4];
    //     odom_msg.twist.twist.linear.z = pos[5];
    //     odom_msg.pose.pose.orientation.w = pos[6];
    //     odom_msg.pose.pose.orientation.x = pos[7];
    //     odom_msg.pose.pose.orientation.y = pos[8];
    //     odom_msg.pose.pose.orientation.z = pos[9];
    //     odom_msg.twist.twist.angular.x = pos[10];
    //     odom_msg.twist.twist.angular.y = pos[11];
    //     odom_msg.twist.twist.angular.z = pos[12];

       
    //     //state.data = //something receive from the rc_pilot
    //     statePub.publish(&odom_msg);
    //     nh.spinOnce();  
    // }
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


