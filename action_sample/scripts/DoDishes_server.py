#! /usr/bin/env python3

import rospy
import actionlib
from action_sample.msg import DoDishesAction, DoDishesGoal, DoDishesFeedback, DoDishesResult

class DishWasherServer(object):
  def __init__(self, action_name):
    self._action_name = action_name
    self._server = actionlib.SimpleActionServer(self._action_name, DoDishesAction, execute_cb=self.execute_cb, auto_start=False)

  def start_server(self):
    self._server.start()
    
  def execute_cb(self, goal):
    rate = rospy.Rate(1)
    
    rospy.loginfo("dishwasher %d is working" %goal.dishwasher_id)
    
    feedback = DoDishesFeedback()
    result = DoDishesResult()
    
    for i in range(10):
      feedback.percent_complete = i * 10
      self._server.publish_feedback(feedback)
      rate.sleep()

      if self._server.is_preempt_requested():
        rospy.loginfo("the goal is canceled")
        result.total_dishes_cleaned = i * 10
        self._server.set_preempted(result)
        return
    
    rospy.loginfo("dishwasher %d finish working" %goal.dishwasher_id)
    result.total_dishes_cleaned = (i+1) * 10
    self._server.set_succeeded(result)


def main():
  rospy.init_node("do_dishes_server")
  
  dish_washer_server = DishWasherServer("do_dishes")
  dish_washer_server.start_server()
  
  rospy.spin()

if __name__ == "__main__":
  main()
