#! /usr/bin/env python3

import rospy
import actionlib

from action_sample.msg import DoDishesAction, DoDishesGoal

def done_cb(state, result):
  rospy.loginfo("%d dishes is clean, state is %s" %(result.total_dishes_cleaned, str(state)))
  rospy.signal_shutdown('Normal shutdown')

def active_cb():
  rospy.loginfo("job is action")
  
def feedback_cb(feedback):
  rospy.loginfo("percent is %f" %feedback.percent_complete)

def main():
  rospy.init_node("do_dishes_client")
  
  client = actionlib.SimpleActionClient("do_dishes", DoDishesAction)
  
  rospy.loginfo("waiting for action server to start")
  client.wait_for_server()
  
  rospy.loginfo("action server started, sending goal")
  goal = DoDishesGoal()
  goal.dishwasher_id = 1

  client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
  
  rospy.sleep(4)
  client.cancel_goal()
  
  rospy.spin()


if __name__ == "__main__":
  main()