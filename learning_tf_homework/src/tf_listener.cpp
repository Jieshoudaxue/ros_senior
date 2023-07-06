#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

void tfPointCb(const ros::TimerEvent& ev, const tf::TransformListener& listener) {


  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_tf_listener");

  ros::NodeHandle nh;
  // The argument inside the parentheses ros::Duration(10) specifies that the cache size of the listener will be 10 seconds. 
  // This means that the listener will store transformations from the last 10 seconds.
  tf::TransformListener listener(ros::Duration(10));

  
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), std::bind(tfPointCb, std::placeholders::_1, std::ref(listener)));



  return 0;
}