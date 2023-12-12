#!/usr/bin/env python3

import rospy
from robot_hunt_maze.srv import *

import actionlib
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  


def image_cb(pose, cmd_pub):
    rospy.loginfo("Target pose: x:%0.6f, y:%0.6f, z:%0.6f" %(pose.position.x, pose.position.y, pose.position.z))
    
    


def main():
  rospy.init_node("move_to_target", anonymous=True)

  move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  
  rospy.loginfo("Waiting for move_base action server...")  
  while move_base_client.wait_for_server(rospy.Duration(5.0)) == 0:
    rospy.loginfo("connected to move base server")
  
  voice_client = rospy.ServiceProxy("str2voice", AddTwoInts)
  rospy.wait_for_service("str2voice")  

  
  cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
  bind_pose_cb = partial(pose_cb, cmd_pub=cmd_pub)
  rospy.Subscriber("/maze_pose", Pose, bind_pose_cb)  


  req = StringToVoice()
  req.data = "找到宝藏了，现在回家"
  resp = voice_client(req)
  if resp.success == True:
        home_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1.0))

        goal = MoveBaseGoal()
        goal.target_pose.pose = home_pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        move_base_client.send_goal(goal)
        
        finished_within_time = move_base_client.wait_for_result()
        
        if move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("go home succeeded")
        else:
            rospy.loginfo("goal failed")        
        

if __name__ == "__main__":
  main()
  
