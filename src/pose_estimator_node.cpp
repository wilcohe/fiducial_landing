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
 // Eigen::Matrix4d glob_pts = {{-0.5, -0.5, 0.5, 0.5}, 
 //               {-0.5, 0.5, -0.5, 0.5}, 
 //               { 0.1, 0.1, 0.1, 0.1}, 
 //               { 1.0, 1.0, 1.0, 1.0}}; 

 // Eigen::Matrix4d glob_pts {{-0.5, -0.5, 0.1, 1.0}, 
 //              {-0.5, 0.5, 0.1, 1.0}, 
 //              { 0.5, -0.5, 0.1, 1.0}, 
 //              { 0.5, 0.5, 1.0, 1.0},}; 

    

  PositionEstimator(ros::NodeHandle nh){
    ROS_INFO("Starting Pose Estimator");
    pos_pub = nh.advertise<geometry_msgs::Pose>("position", 100, true);
    ROS_INFO("Pose publisher initialized"); 
    detect_sub = nh.subscribe("/tag_detections", 10, &PositionEstimator::positionCallback, this);
    ROS_INFO("Detection subscriber initialized"); 
    for (int i = 0; i < 4; i++){
      glob_pts(i, 0) = pow(-1.0, i)*0.5;
      if (i < 2)
        glob_pts(i, 1) = 0.5;
      else
         glob_pts(i, 1) = 0.5;
      glob_pts(i, 2) = 0.1; 
      glob_pts(i, 3) = 1.0; 
    }

    // glob_pts << -0.5, -0.5, 0.1, 1.0, -0.5, 0.5, 0.1, 1.0, 0.5, -0.5, 0.1, 1.0, 0.5, 0.5, 0.1, 1.0; 
    ROS_INFO("Finished Pose Estimator");
  }

 private: 

  // Eigen::MatrixXf glob_pts;
  Eigen::Matrix4f glob_pts;  
  ros::Publisher pos_pub; 
  ros::Subscriber detect_sub; 

  void positionCallback(const apriltag_ros::AprilTagDetectionArray det){

    // ROS_INFO("Position Called");

   int num = det.detections.size();

   Eigen::MatrixXf camera = Eigen::MatrixXf::Zero(4, 4); 

   // if (num < 3)
   //  return ;

   geometry_msgs::Point centroid; 

   // for (auto& det.detections : *it){
   // for (auto it det.begin(); it < det.end(); ++it){
   for (int i = 0; i < num; i++){

    centroid.x += det.detections[i].pose.pose.pose.position.x/i;
    centroid.y += det.detections[i].pose.pose.pose.position.y/i; 
    centroid.z += det.detections[i].pose.pose.pose.position.z/i; 

    // camera(0, i) = det.detections[i].pose.pose.pose.position.x;
    // camera(1, i) = det.detections[i].pose.pose.pose.position.y;
    // camera(2, i) = det.detections[i].pose.pose.pose.position.z;
    // camera(3, i) = 1;

    camera(i, 0) = det.detections[i].pose.pose.pose.position.x; 
    camera(i, 1) = det.detections[i].pose.pose.pose.position.y; 
    camera(i, 2) = det.detections[i].pose.pose.pose.position.z; 
    camera(i, 3) = 1.0; 

   }


   Eigen::Matrix4f sol = camera.ldlt().solve(glob_pts); 

   centroid.x = sol(3, 0); 
   centroid.y = sol(3, 1); 
   centroid.z = sol(3, 2); 

   pos_pub.publish(centroid); 

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
