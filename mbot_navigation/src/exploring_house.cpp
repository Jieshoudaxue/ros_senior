#include <ros/ros.h>
#include <list>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_test");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

  ROS_INFO("Waiting for move_base action server...");  
  move_base_client.waitForServer();
  ROS_INFO("connected to move base server");

  std::vector<geometry_msgs::Pose> target_list;
  target_list.push_back(createPose(6.543, 4.779, 0.000, 0.000, 0.000, 0.645, 0.764));
  target_list.push_back(createPose(5.543, -4.779, 0.000, 0.000, 0.000, 0.645, 0.764));
  target_list.push_back(createPose(-5.543, 4.779, 0.000, 0.000, 0.000, 0.645, 0.764));
  target_list.push_back(createPose(-5.543, -4.779, 0.000, 0.000, 0.000, 0.645, 0.764));

  for (uint8_t i = 0; i < target_list.size(); i ++) {
    ros::Time start_time = ros::Time::now();

    ROS_INFO("going to %u goal, position: (%f, %f)", i, target_list[i].position.x, target_list[i].position.y);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose = target_list[i];

    move_base_client.sendGoal(goal);

    move_base_client.waitForResult();

    if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ros::Duration running_time =  ros::Time::now() - start_time;
      ROS_INFO("go to %u goal succeeded, running time %f sec", i, running_time.toSec());
    } else {
      ROS_INFO("goal failed");
    }

  }



  return 0;
}