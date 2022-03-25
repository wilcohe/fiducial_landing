#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <random>

class PositionEstimator{
 public:

  PositionEstimator(ros::NodeHandle nh){
    ROS_INFO("Starting Pose Estimator");
    pos_pub = nh.advertise<geometry_msgs::Pose>("position", 100, true);
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
        glob_pts(i, 1) = -0.5

      glob_pts(i, 2) = 0.1; 
      glob_pts(i, 3) = 1.0; 
    }

    ROS_INFO("Finished Pose Estimator");
  }

 private: 
  Eigen::Matrix4f glob_pts;  
  ros::Publisher pos_pub; 
  ros::Subscriber detect_sub; 

  void positionCallback(const apriltag_ros::AprilTagDetectionArray det){

    ROS_INFO("Position Called");

   int num = det.detections.size();

   Eigen::Matrix4f camera; 

   if (num < 4){
    ROS_INFO("Not enough detections.");
    return ;
  }

   geometry_msgs::Pose centroid; 


   for (int i = 0; i < num; i++){

    if (det.detections[i].id.size() > 0)
      continue; 

    int id = det.detections[i].id[0] - 1 ; 

    camera(id, 0) = det.detections[i].pose.pose.pose.position.x; 
    camera(id, 1) = det.detections[i].pose.pose.pose.position.y; 
    camera(id, 2) = det.detections[i].pose.pose.pose.position.z; 
    camera(id, 3) = 1.0; 

   }

   ROS_INFO("Initialized Camera Matrix. Solving...");

   Eigen::Matrix4f sol = camera.ldlt().solve(glob_pts); 

   centroid.position.x = sol(3, 0); 
   centroid.position.y = sol(3, 1); 
   centroid.position.z = sol(3, 2); 

   ROS_INFO("Generated solution");
   ROS_INFO("X: %f", sol(3, 0)); 
   ROS_INFO("Y: %f", sol(3, 1)); 
   ROS_INFO("Z: %f", sol(3, 2)); 

   pos_pub.publish(centroid);

   ROS_INFO("Published");  

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
