#include  "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Test {
  public: 
  Test(ros::NodeHandle nh) {
    ros::param::get("/goal_state", state);
    currentPoseSub = nh.subscribe("/pose_estimator/position", 1, &Test::getCurrent, this);
    aprilStateSub = nh.subscribe("/april_state", 1, &&TrajGenerator::activate, this);
    goalPosePub = nh.advertise<nav_msgs::Odometry>("/goal", 1, true);
  }

  private:
  ros::Subscriber currentPoseSub;
  ros::Subscriber aprilStateSub;
  ros::Publisher goalPosePub;
  Eigen::Affine3d current_pose_;
  char state;

  void getCurrent(const nav_msgs::Odometry::ConstPtr& odom) {
    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);
  }

  void activate(std_msgs::Bool state_msg) {
    if (state_msg.data) {
      publishGoal();
    }
  }

  void publishGoal() {
    nav_msgs::Odometry goal;
    tf2::Quaternion goal_quat;
    switch(state) {
      case 'l': // land
        goal.pose.pose.position.x = 0.0; 
        goal.pose.pose.position.y = 0.0; 
        goal.pose.pose.position.z = 0.0; 
        goal_quat.setRPY(0, 0, 0);
        goal.pose.pose.orientation = tf2::toMsg(goal_quat);
      case 'g': // guide to above landing zone 
        goal.pose.pose.position.x = 0.0; 
        goal.pose.pose.position.y = 0.0; 
        goal.pose.pose.position.z = 0.75; 
        goal_quat.setRPY(0, 0, 0);
        goal.pose.pose.orientation = tf2::toMsg(goal_quat);
      case 'h': // hover in place
        tf::poseEigenToMsg(current_pose_, goal.pose.pose)
        goal.pose.pose.position.z = 0.75; 
    }
    goalPosePub.publish(goal);
    ROS_INFO("published goal");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_test_node");
  ros::NodeHandle nh;

  Test test(nh);

  ros::spinOnce();
  return 0;
}