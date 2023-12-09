#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <robot_hunt_maze/StringToVoice.h>

#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


class RobotController {
public:
  RobotController() {
    ROS_INFO("RobotController Constructor");
  }

  ~RobotController() {
    ROS_INFO("RobotController Destructor");
  }

  int Init(ros::NodeHandle& nh) {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    client_ = nh.serviceClient<robot_hunt_maze::StringToVoice>("str2voice");
    return 0;
  }

  void Start() {
    robot_hunt_maze::StringToVoice::Request req;
    robot_hunt_maze::StringToVoice::Response resp;
    req.data = "找到宝藏了，现在回家";

    bool ok = client_.call(req, resp);
    if (ok) {

      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

      ROS_INFO("Waiting for move_base action server...");  
      move_base_client.waitForServer();
      ROS_INFO("connected to move base server");

      geometry_msgs::Pose home_pose = createPose(0, 0, 0, 0, 0, 0, 1.0);
      

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose = home_pose;

      move_base_client.sendGoal(goal);

      move_base_client.waitForResult();

      if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("go home succeeded");
      } else {
        ROS_INFO("goal failed");
      }   


    } else {
      ROS_ERROR("failed to send str2voice service");
    }
  }

  geometry_msgs::Pose createPose(double px, double py, double pz, double ox, double oy, double oz, double ow) {
    geometry_msgs::Pose pose;
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    pose.orientation.x = ox;
    pose.orientation.y = oy;
    pose.orientation.z = oz;
    pose.orientation.w = ow;
    return pose;  
  }

private:
  ros::Publisher cmd_pub_;
  ros::ServiceClient client_;
};


int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "voice_controller");
  ros::NodeHandle nh;

  RobotController rc;
  rc.Init(nh);

  printf("this is a robot controller for hunt maze\n");
  rc.Start();

  ros::spin();

  return 0;
}
