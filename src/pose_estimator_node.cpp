#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <random>

class PositionEstimator{
 public:

  Eigen::Vector3f position; 
  Eigen::Quaterniond orientation; 

  PositionEstimator(ros::NodeHandle nh){
    ROS_INFO("Starting Pose Estimator");
    pos_pub = nh.advertise<nav_msgs::Odometry>("position", 100, true);
    ROS_INFO("Pose publisher initialized"); 
    detect_sub = nh.subscribe("/tag_detections", 10, &PositionEstimator::positionCallback, this);
    ROS_INFO("Detection subscriber initialized"); 
    for (int i = 0; i < 4; i++){
      if (i < 2)
        glob_pts(i, 0) = 0.5;
      else
         glob_pts(i, 0) = -0.5;
      if (i < 1 || i > 2)
        glob_pts(i, 1) = 0.5; 
      else
        glob_pts(i, 1) = -0.5;

      glob_pts(i, 2) = 0.1; 
      glob_pts(i, 3) = 1.0; 
    }

    ROS_INFO("Finished Pose Estimator");

    avgPoses(curr_detect, &position, &orientation); 

    pubPoses(position, orientation); 


  }

 private: 
  Eigen::Matrix4f glob_pts;  
  ros::Publisher pos_pub; 
  ros::Subscriber detect_sub; 
  apriltag_ros::AprilTagDetectionArray curr_detect; 

  int singleTagDetect(const apriltag_ros::AprilTagDetection det, Eigen::Vector3f curr_p, Eigen::Quaterniond curr_o){

    if (det.id.size()  > 1)
          return 1; 
    
    int id = det.id[0];

    Eigen::Quaterniond rpy; 

    rpy.w() = det.pose.orientation.w; 
    rpy.x() = det.pose.orientation.x; 
    rpy.y() = det.pose.orientation.y;
    rpy.z() = det.pose.orientation.z;

    Eigen::vector3f glob = glob_pts.row(i-1).seq(0,3).transpose();
    Eigen::Vector3f pose_c(det.pose.position.x, det.pose.position.y, det.pose.position.z);

    curr_p = glob + pose_c;
    curr_o = rpy;

    return 0;

  }

  void avgPoses(apriltag_ros::AprilTagDetectionArray det,
                Eigen::Vector3f curr_p, Eigen::Quaterniond curr_o){

    int num_detect = AprilTagDetectionArray.detections.size();
    int num = 0;

    for (int i = 0; i < num_detect; i++){

      Eigen::Vector3f curr_p; 
      Eigen::Quaterniond curr_o; 

      if (!SingleTagDetect(det[i], &curr_p, &curr_o)){

        curr_p(0) += curr_ps[i](0);
        curr_p(1) += curr_ps[i](1);
        curr_p(2) += curr_ps[i](2);

        curr_o.w() += curr_os[i].w;
        curr_o.x() += curr_os[i].x;
        curr_o.y() += curr_os[i].y;
        curr_o.z() += curr_os[i].z;

        num++;
      }

    }

    curr_p(0) += curr_p(0)/num;
    curr_p(1) += curr_p(1)/num;
    curr_p(2) += curr_p(2)/num;

    curr_o.w() += curr_o.w/num;
    curr_o.x() += curr_o.x/num;
    curr_o.y() += curr_o.y/num;
    curr_o.z() += curr_o.z/num;

  }


  void pubPoses(Eigen::Vector3f curr_p, Eigen::Quaterniond curr_o){

    nav_msgs::Odometry odom; 

    odom.pose.pose.position.x = curr_p(0);
    odom.pose.pose.position.y = curr_p(1); 
    odom.pose.pose.position.z = curr_p(2);

    odom.pose.pose.orientation.w = curr_o.w; 
    odom.pose.pose.orientation.x = curr_o.x;
    odom.pose.pose.orientation.y = curr_o.y; 
    odom.pose.pose.orientation.z = curr_o.z; 

    pose_pub.publish(odom); 

  }

  void positionCallback(const apriltag_ros::AprilTagDetectionArray det){

    ROS_INFO("Position Called");

    curr_detect = det;  

  }

}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimator");
  ROS_INFO("Initializing.");

  ros::NodeHandle nh("~");

  PositionEstimator pose_estimator(nh);  


 // ROS_INFO_THROTTLE(1.0, "Pose estimator initialized. "); 
  ros::spin(); 

}
