#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_sample/DoDishesAction.h"

void execute(const action_sample::DoDishesGoalConstPtr& goal, actionlib::SimpleActionServer<action_sample::DoDishesAction>* server) {
  ros::Rate rate(1);

  action_sample::DoDishesFeedback feedback;
  action_sample::DoDishesResult result;
  ROS_INFO("dishwasher %d is working", goal->dishwasher_id);

  int i = 0;
  for (i = 0; i < 10; i++) {
    feedback.percent_complete = i * 10;
    server->publishFeedback(feedback);
    rate.sleep();

    if (server->isPreemptRequested()) {
      ROS_INFO("The goal is canceled");
      result.total_dishes_cleaned = i*10;
      server->setPreempted(result);
      return;
    }
  }

  ROS_INFO("dishwasher %d finish working", goal->dishwasher_id);
  result.total_dishes_cleaned = i*10;
  server->setSucceeded(result);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle nh;

  actionlib::SimpleActionServer<action_sample::DoDishesAction> server(nh, "do_dishes", std::bind(&execute, std::placeholders::_1, &server), false);

  server.start();

  ros::spin();

  return 0;
}