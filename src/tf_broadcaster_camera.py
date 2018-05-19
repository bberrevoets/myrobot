#!/usr/bin/env python
'''raspi_camera TF'''
import os
import roslib
import rospy
import tf
roslib.load_manifest('myrobot')


def talker():
    '''camera tf'''
    rospy.init_node('raspicam_tf', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        br.sendTransform((0.05, 0.0, 0.15),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(), "raspicam", "base_link")
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
