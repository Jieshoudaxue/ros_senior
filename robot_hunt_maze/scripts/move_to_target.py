#!/usr/bin/env python3

import rospy
from functools import partial
from robot_hunt_maze.srv import *

import actionlib
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  


def pose_cb(pose, cmd_pub):
  rospy.loginfo("Target pose: x:%0.6f, y:%0.6f, z:%0.6f" %(pose.position.x, pose.position.y, pose.position.z))
    
    


def main():
  rospy.init_node("move_to_target", anonymous=True)



  cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
  bind_pose_cb = partial(pose_cb, cmd_pub=cmd_pub)
  rospy.Subscriber("/maze_pose", Pose, bind_pose_cb)  


  voice_client = rospy.ServiceProxy("str2voice", StringToVoice)
  rospy.wait_for_service("str2voice")  

  req = StringToVoiceRequest()
  req.data = "找到宝藏了，现在回家"
  resp = voice_client(req)
  print("send txt %s, resp %d" %(req.data, resp.success))

  rospy.spin()

if __name__ == "__main__":
  main()
  
