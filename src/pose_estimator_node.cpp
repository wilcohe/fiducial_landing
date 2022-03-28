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
        glob_pts(i, 1) = -0.5;

      glob_pts(i, 2) = 0.1; 
      glob_pts(i, 3) = 1.0; 
    }

    ROS_INFO("Finished Pose Estimator");
  }

 private: 
  Eigen::Matrix4f glob_pts;  
  ros::Publisher pos_pub; 
  ros::Subscriber detect_sub; 
  apriltag_ros::AprilTagDetectionArray curr_detect; 

  void singleTagDetect(const apriltag_ros::AprilTagDetection det, Eigen::Vector3f& curr_p, Eigen::Quaternion& curr_o){

    if (det.id.size()  > 1)
          return; 
    
    int id = det.id[0];

    // Eigen::MatrixXf glob = glob_pts.row(i-1).seq(0,3).transpose();

    // Eigen::Quaternion rpy; 

    // rpy.w() = det.pose.orientation.w; 
    // rpy.x() = det.pose.orientation.x; 
    // rpy.y() = det.pose.orientation.y;
    // rpy.z() = det.pose.orientation.z; 

    // Eigen::Matrix3d Rt = rpy.normalized().toRotationMatrix().transpose();

    // Eigen::Matrix4d TCA = Eigen::Matrix4d::Zero(); 

    // TCA.block<3, 3>(0, 0) = Rt;

    // // TCA.block<3,1>(0,3) = -Rt*glob;

    // Eigen::Vector3f pose_c(det.pose.position.x, det.pose.position.y, det.pose.position.z);

    // TCA.block<3,1>(0,3) = -Rt*pose_c;

    // TCA(3,3) = 1;

    // Eigen::Matrix4f TAO = Eigen::Matrix4f::Identity(); 

    // TAO.block<3,1>(0,3) = glob;

    // Eigen::Matrix4f TCO = TAO*TCA;

    // Quaternion

    Eigen::Quaternion rpy; 

    rpy.w() = det.pose.orientation.w; 
    rpy.x() = det.pose.orientation.x; 
    rpy.y() = det.pose.orientation.y;
    rpy.z() = det.pose.orientation.z;

    Eigen::vector3f glob = glob_pts.row(i-1).seq(0,3).transpose();
    Eigen::Vector3f pose_c(det.pose.position.x, det.pose.position.y, det.pose.position.z);

    curr_p = glob + pose_c;
    curr_o = rpy;

  }

  void avgPoses(std::vector<Eigen::Vector3d> curr_ps, std::vector<Eigen::Quaternion> curr_os,
                Eigen::Vector3f& curr_p, Eigen::Quaternion& curr_o){

    int num_p = curr_ps.size();
    int nump_o = curr_os.size(); 

    if (num_p != num_o){
      ROS_WARN("Number of positions is not equal to number of orientations.");
      throw std::runtime_error("Array sizes not equal");
    }

    for (int i = 0; i < curr_ps.size(); i++){

      curr_p(0) += curr_ps[i](0)/num_p;
      curr_p(1) += curr_ps[i](1)/num_p;
      curr_p(2) += curr_ps[i](2)/num_p;

      curr_o.w() += curr_os[i].w/num_o;
      curr_o.x() += curr_os[i].x/num_o;
      curr_o.y() += curr_os[i].y/num_o;
      curr_o.z() += curr_os[i].z/num_o;

    }

  }


  void pubPoses(Eigen::Vector3f curr_p, Eigen::Quaternion curr_o){

    geometry_msgs::Pose pose; 

    pose.position.x = curr_p(0);
    pose.position.y = curr_p(1); 
    pose.position.z = curr_p(2);

    pose.orientation.w = curr_o.w; 
    pose.orientation.x = curr_o.x;
    pose.orientation.y = curr_o.y; 
    pose.orientation.z = curr_o.z; 

    pose_pub.publish(pose); 

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

  while(ros::ok()){



  }


 // ROS_INFO_THROTTLE(1.0, "Pose estimator initialized. "); 
  ros::spin(); 

}
