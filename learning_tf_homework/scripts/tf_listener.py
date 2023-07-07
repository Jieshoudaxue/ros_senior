#! /usr/bin/env python3

import rospy
import tf
import random
from functools import partial

from geometry_msgs.msg import PointStamped

def tfPointCb(event, listener):
  laser_point = PointStamped()
  
  laser_point.header.frame_id = "base_laser"
  laser_point.header.stamp = rospy.Time(0)

  laser_point.point.x = random.random()
  laser_point.point.y = random.random()
  laser_point.point.z = random.random()
  
  try:
    (translation, rotation) = listener.lookupTransform("base_link", "base_laser", rospy.Time(0))    
    rospy.loginfo("base_laser to base_link translation transform: (%.2f, %.2f, %.2f)" \
          %(translation[0], translation[1], translation[2]))
    
    base_point = listener.transformPoint("base_link", laser_point)
    rospy.loginfo("base_laser: (%.2f, %.2f, %.2f) --> base_link: (%.2f, %.2f, %.2f) at time %.2f" \
            %(laser_point.point.x, laser_point.point.y, laser_point.point.z,
            base_point.point.x, base_point.point.y, base_point.point.z,
            base_point.header.stamp.to_sec()))
  except Exception as e:
    rospy.logerr("---> %s" %e)


def main():
  rospy.init_node("robot_tf_listener")
  
  tf_listener = tf.TransformListener(rospy.Duration(10))

  bind_tf_point_cb = partial(tfPointCb, listener=tf_listener)

  timer = rospy.Timer(rospy.Duration(1), bind_tf_point_cb)

  rospy.spin()

if __name__ == "__main__":
  main()
