#! /usr/bin/env python3

import rospy
import tf

def main():
  rospy.init_node("robot_tf_broadcaster")
  
  tf_br = tf.TransformBroadcaster()

  rate = rospy.Rate(100)
  while not rospy.is_shutdown():
    translate_date = (0.1, 0.0, 0.2)
    rotation_data = (0, 0, 0, 1)
    tf_br.sendTransform(translate_date, rotation_data, rospy.Time.now(), "base_laser", "base_link")
    
    rate.sleep()

if __name__ == "__main__":
  main()
