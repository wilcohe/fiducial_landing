#include  "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Test {
  public: 
  Test(ros::NodeHandle nh) {
    ros::Publisher currentStatePub = nh.advertise<nav_msgs::Odometry>("/current_state", 1);
    ros::Publisher goalPosePub = nh.advertise<nav_msgs::Odometry>("/goal", 1);
    publishCurrent();
    publishGoal();
  }

  void publishCurrent() {
    nav_msgs::Odometry curr;
    curr.pose.pose.position.x = 0.0; 
    curr.pose.pose.position.y = 0.0; 
    curr.pose.pose.position.z = 0.0; 
    tf2::Quaternion curr_quat;
    curr_quat.setRPY(0, 0, 0);
    curr.pose.pose.orientation = tf2::toMsg(curr_quat)
    currentStatePub.publish(curr);
  }

  void publishGoal() {
    nav_msgs::Odometry goal;
    goal.pose.pose.position.x = 1.0; 
    goal.pose.pose.position.y = 1.0; 
    goal.pose.pose.position.z = 2.0; 
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0, 0, M_PI/2);
    goal.pose.pose.orientation = tf2::toMsg(goal_quat)
    currentStatePub.publish(goal);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_test_node");
  ros::NodeHandle nh;

  Test test(nh);

  ros::spinOnce();
  return 0;
}