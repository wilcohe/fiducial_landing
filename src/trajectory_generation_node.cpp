#include  "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <mav_msgs/eigen_mav_msgs.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TrajGenerator {
  
  public:
  TrajGenerator(ros::NodeHandle nh) {
    currentPoseSub = nh.subscribe("/current_state", 10, &TrajGenerator::currentStateCB, this);
    goalPoseSub = nh.subscribe("/goal", 10, &TrajGenerator::goalPoseCB, this);
    
    desiredStatePub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 1);
    desiredStateTimer = nh.createTimer(ros::Rate(5), &TrajGenerator::publishDesiredState, this);
    desiredStateTimer.start();
  }

  
  private:
  ros::Subscriber currentPoseSub;
  ros::Subscriber goalPoseSub;
  ros::Publisher desiredStatePub;
  ros::Timer desiredStateTimer; 

  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  
  Eigen::Affine3d goal_pose_;

  const float max_v = 1.0;
  const float max_a = 2.0;
  const float max_ang_v = 1.0;
  const float max_ang_a = 2.0;

  const int D = 3;

  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;
  ros::Time trajectoryStartTime;


  void currentStateCB(const nav_msgs::Odometry::ConstPtr& odom) {
    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
    tf::vectorMsgToEigen(odom->twist.twist.angular, current_angular_velocity_);
  }


  void goalPoseCB(const nav_msgs::Odometry::ConstPtr& odom) {
    // store goal pose
    tf::poseMsgToEigen(odom->pose.pose, goal_pose_);
    updateTraj();
  }


  void updateTraj() {
    trajectory.clear();
    yaw_trajectory.clear();
    
    Eigen::Vector3d start_pos, start_vel;
    float start_yaw
    start_pos << current_pose_.translation();
    start_vel << current_velocity_;

    // start_yaw << mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation());

    Eigen::Vector3d goal_pos, goal_vel;
    goal_pos << goal_pose_.translation();
    goal_vel << 0.0, 0.0, 0.0;

    // Make trajectory waypoints
    mav_trajectory_generation::Vertex start(D), end(D);
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start_yaw(1), end_yaw(1);
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    start.makeStartOrEnd(start_pos, derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel);
    vertices.push_back(start);

    start_yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, mav_msgs::yawFromQuaternion((Eigen::Quaterniond)current_pose_.rotation()));
    yaw_vertices.push_back(start_yaw);

    end.makeStartOrEnd(goal_pos, derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel);
    vertices.push_back(end);
    
    end_yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, mav_msgs::yawFromQuaternion((Eigen::Quaterniond)goal_pose_.rotation()));
    yaw_vertices.push_back(end_yaw);

    segment_times = estimateSegmentTimes(vertices, max_v, max_a);

    // initialize optimizer
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, derivative_to_optimize);


    // add max constraints
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    // solve for trajectory
    opt.solveLinear();
    yaw_opt.solveLinear();

    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);

    trajectoryStartTime = ros::Time::now();
  }


  void publishDesiredState(ros::TimerEvent const& ev) {
    if (trajectory.empty()) return;

    trajectory_msgs::MultiDOFJointTrajectoryPoint setpoint;
    setpoint.time_from_start = ros::Time::now() - trajectoryStartTime;

    double sampling_time = setpoint.time_from_start.toSec(); 
    if (sampling_time > trajectory.getMaxTime()) {
      sampling_time = trajectory.getMaxTime();
    }

    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector3d des_position = trajectory.evaluate(sampling_time, POSITION);
    Eigen::Vector3d des_velocity = trajectory.evaluate(sampling_time, VELOCITY);
    Eigen::Vector3d des_accel = trajectory.evaluate(sampling_time, ACCELERATION);
    Eigen::VectorXd des_orientation = yaw_trajectory.evaluate(sampling_time, ORIENTATION);

    geometry_msgs::Transform t;
    t.translation.x = des_position(0);
    t.translation.y = des_position(1);
    t.translation.z = des_position(2);

    tf2::Quaternion des_quat;
    des_quat.setRPY(0, 0, des_orientation(0));
    t.rotation = tf2::toMsg(des_quat);

    setpoint.transforms.push_back(t);

    geometry_msgs::Twist t2;
    t2.linear.x = des_velocity(0);
    t2.linear.y = des_velocity(1);
    t2.linear.z = des_velocity(2);
    t2.angular.x = 0;
    t2.angular.y = 0;
    t2.angular.z = 0;
    setpoint.velocities.push_back(t2);

    geometry_msgs::Twist t3;
    t3.linear.x = des_accel(0);
    t3.linear.y = des_accel(1);
    t3.linear.z = des_accel(2);
    t3.angular.x = 0;
    t3.angular.y = 0;
    t3.angular.z = 0;
    setpoint.accelerations.push_back(t3);

    desiredStatePub.publish(setpoint);
  }

};


int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;

  TrajGenerator trajGenerator(nh);

  ros::spin();
  return 0;
}
