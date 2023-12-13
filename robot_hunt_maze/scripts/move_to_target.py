#!/usr/bin/env python3

import rospy
from functools import partial
from robot_hunt_maze.srv import *

import actionlib
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from std_msgs.msg import Int8

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2
GET_TARGET_SIZE     = 100000

g_status_flag       = STATUS_EXPLORING


def pose_cb(pose, state_pub, cmd_pub, voice_client):
  global g_status_flag
  # rospy.loginfo("Target pose: x:%0.6f, y:%0.6f, z:%0.6f" %(pose.position.x, pose.position.y, pose.position.z))
  if g_status_flag == STATUS_EXPLORING:
    g_status_flag = STATUS_CLOSE_TARGET
    
    state_msg = Int8()
    state_msg.data = g_status_flag
    state_pub.publish(state_msg)

    req = StringToVoiceRequest()
    req.data = "发现宝藏了，跑过去拿"
    resp = voice_client(req)
    rospy.loginfo("send txt %s, resp %d" %(req.data, resp.success))    
    
  elif g_status_flag == STATUS_CLOSE_TARGET and pose.position.z > GET_TARGET_SIZE:
    g_status_flag = STATUS_GO_HOME
    
    state_msg = Int8()
    state_msg.data = g_status_flag
    state_pub.publish(state_msg)

    req = StringToVoiceRequest()
    req.data = "拿到宝藏了，回家数数"
    resp = voice_client(req)
    rospy.loginfo("send txt %s, resp %d" %(req.data, resp.success))        
    
    
  elif g_status_flag == STATUS_CLOSE_TARGET:
    vel_msg = Twist()
    vel_msg.linear.x = (200000 - pose.position.z) / 200000 * 0.3
    vel_msg.angular.z = (600 - pose.position.x) / 600 * 0.3

    cmd_pub.publish(vel_msg)
  
def main():
  rospy.init_node("move_to_target", anonymous=True)

  cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
  state_pub = rospy.Publisher("/state_cmd", Int8, queue_size=1)
  
  voice_client = rospy.ServiceProxy("str2voice", StringToVoice)
  rospy.wait_for_service("str2voice")  


  bind_pose_cb = partial(pose_cb, state_pub=state_pub, cmd_pub=cmd_pub, voice_client=voice_client)
  rospy.Subscriber("/maze_pose", Pose, bind_pose_cb)  

  rospy.spin()

if __name__ == "__main__":
  main()
  
