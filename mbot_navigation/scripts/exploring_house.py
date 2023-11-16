#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  


def main():
  rospy.init_node("move_test", anonymous=True)
  
  move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  
  rospy.loginfo("Waiting for move_base action server...")  
  
  while move_base_client.wait_for_server(rospy.Duration(5.0)) == 0:
    rospy.loginfo("connected to move base server")
  
  target_list = []
  target_list.append(Pose(Point(6.543, 4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  target_list.append(Pose(Point(5.543, -4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  target_list.append(Pose(Point(-5.543, 4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  target_list.append(Pose(Point(-5.543, -4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  

  for i, target in enumerate(target_list):
    start_time = rospy.Time.now()  
    
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    
    rospy.loginfo("going to {0} goal, {1}".format(i, str(target)))
    
    move_base_client.send_goal(goal)
    
    finished_within_time = move_base_client.wait_for_result(rospy.Duration(300))
    
    if not finished_within_time:
      move_base_client.cancel_goal()
      rospy.loginfo("time out, failed to goal")
    else:
      running_time = (rospy.Time.now() - start_time).to_sec()
      if move_base_client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("go to {0} goal succeeded, run time: {1} sec".format(i, running_time))
      else:
        rospy.loginfo("goal failed")
    

if __name__ == "__main__":
  main()