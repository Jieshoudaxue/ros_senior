#include <actionlib/client/simple_action_client.h>
#include "action_sample/DoDishesAction.h"

void doneCb(const actionlib::SimpleClientGoalState& state,
            const action_sample::DoDishesResultConstPtr& result) {
  ROS_INFO("%d dishes is clean, state is %s", result->total_dishes_cleaned, state.toString().c_str());
  ros::shutdown();
}

void activeCb() {
  ROS_INFO("job is active");
}

void feedbackCb(const action_sample::DoDishesFeedbackConstPtr& feedback) {
  ROS_INFO("percent is %f", feedback->percent_complete);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "do_dishes_client");

  actionlib::SimpleActionClient<action_sample::DoDishesAction> client("do_dishes", true);

  ROS_INFO("waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("action server started, sending goal.");

  action_sample::DoDishesGoal goal;
  goal.dishwasher_id = 1;

  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  // sleep(4);
  // client.cancelGoal();

  ros::spin();

  return 0;
}