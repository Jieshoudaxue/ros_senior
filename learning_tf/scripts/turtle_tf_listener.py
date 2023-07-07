#！/usr/bin/env python3

import roslib
import rospy
import math
import tf

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

def main():
  rospy.init_node("turtle_tf_listener")
  
  rospy.wait_for_service("spawn")
  client = rospy.ServiceProxy("spawn", Spawn)
  
  try:
    req = SpawnRequest()
    resp = client(req)
    rospy.loginfo("spawned a turtle name %s" %resp.name)
  except rospy.ServiceException as e:
      rospy.loginfo("service call fail: %s" %e)

  cmd_pub= rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)
  
  tf_listener = tf.TransformListener()
  
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    try:
      (translation, rotation) = tf_listener.lookupTransform("/turtle2", "turtle1", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    cmd_vel = Twist()
    cmd_vel.linear.x= 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    cmd_vel.angular.z = 4 * math.atan2(translation[1], translation[0])

    # 下面的改动，可以让turtle1控制turtle2的转向，即turtle2跟随turtle1转，但turtle2追不上turtle1
    # quaternion = rotation
    # # euler = (roll, pitch, yaw)
    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # cmd_vel.angular.z = 4 * euler[2]
    
    cmd_pub.publish(cmd_vel)
    
    rate.sleep()

if __name__ == "__main__":
  main()