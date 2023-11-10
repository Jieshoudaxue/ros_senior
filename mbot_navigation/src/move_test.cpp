#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_test");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

  while (move_base_client.waitForServer(ros::Duration(5.0)) != 0) {
    ROS_INFO("Waiting for move_base action server...");  
  }
  ROS_INFO("connected to move base server");

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = -5.543;
  goal.target_pose.pose.position.y = -4.779;

  goal.target_pose.pose.orientation.z = 0.645;
  goal.target_pose.pose.orientation.w = 0.764;

  move_base_client.sendGoal(goal);

  move_base_client.waitForResult();

  if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("goal succeeded");
  } else {
    ROS_INFO("goal failed");
  }


  return 0;
}