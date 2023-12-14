#!/usr/bin/env python3

import rospy
import random
import actionlib
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from std_msgs.msg import Int8
from functools import partial

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2

target_list = []
target_list.append(Pose(Point(0, 8, 0), Quaternion(0, 0, 0, 1.0)))
target_list.append(Pose(Point(8, 8, 0), Quaternion(0, 0, 0, 1.0)))
target_list.append(Pose(Point(8, 0, 0), Quaternion(0, 0, 0, 1.0)))
target_list.append(Pose(Point(2.5, 4.8, 0), Quaternion(0, 0, 0, 1.0)))
target_list.append(Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1.0)))

def state_cb(state, move_base_client):
  rospy.loginfo("receive state is %d" %state.data)
  
  if state.data == STATUS_CLOSE_TARGET:
    move_base_client.cancel_goal()
    rospy.loginfo("stop exploring")
  elif state.data == STATUS_GO_HOME:
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_list[-1]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()    

    move_base_client.send_goal(goal)
    move_base_client.wait_for_result(rospy.Duration(300))
    if move_base_client.get_state() == GoalStatus.SUCCEEDED:
      rospy.loginfo("go home successed")


def main():
  rospy.init_node("exploring_maze", anonymous=True)
  move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  
  rospy.loginfo("Waiting for move_base action server...")  
  
  while move_base_client.wait_for_server(rospy.Duration(5.0)) == 0:
    rospy.loginfo("connected to move base server")

  bind_state_cb = partial(state_cb, move_base_client=move_base_client)
  rospy.Subscriber("/state_cmd", Int8, bind_state_cb)

  while not rospy.is_shutdown():
    start_time = rospy.Time.now()  
    
    i = random.SystemRandom().randint(0, 4)
    
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_list[i]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    
    rospy.loginfo("going to {0} goal, {1}".format(i, str(target_list[i])))
    
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
    if move_base_client.get_state() == GoalStatus.SUCCEEDED:
      running_time = (rospy.Time.now() - start_time).to_sec()
      rospy.loginfo("go to {0} goal succeeded, run time: {1} sec".format(i, running_time))
    else:
      break

  rospy.spin()    

if __name__ == "__main__":
  main()