#include <ros/ros.h>
#include <geometry_msgs>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <random>

using namespace grid_map;

void positionCallback(???){

  

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle nh("~");
  ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3>("position", 1, true);
  ros::Subscriber detect_sub = nh.subscribe("tag_detections", 10, )

  publisher.publish(message); 
  ROS_INFO_THROTTLE(1.0, "Pose estimator initialized. "); 

  rate.sleep(); 


}
