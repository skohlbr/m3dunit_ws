#!/usr/bin/env python
import roslib
import rospy
import tf
import geometry_msgs.msg
import os
from sensor_msgs.msg import JointState
import math
angle = 0;



def joinStateCallback (data):

    if (data.name[0] == "m3d_laser_rotation_joint"):
        #print data
        angle = data.position[0]
        broadcaster = tf.TransformBroadcaster()

        q=tf.transformations.quaternion_from_euler(0,-math.pi/2,angle)
        broadcaster.sendTransform((0.0285, 0 , 0.04), (0,0,0,1),
                    rospy.Time.now(), m3d_front_frame_id, m3d_frame_id)
        broadcaster.sendTransform((-0.0835, 0, 0.1835), q,
                    rospy.Time.now(), m3d_rot_frame_id,  m3d_frame_id)

if __name__ == '__main__':

  rospy.init_node('m3d_driver_sim')
  encoderOffset         = rospy.get_param ("encoderOffset", 0 )
  m3d_frame_id          = rospy.get_param ("m3d_frame_id", "m3d_sim/m3d_link" )
  m3d_front_frame_id    = rospy.get_param ("m3d_front_frame_id", "m3d_sim/m3d_front_laser_link" )
  m3d_rot_frame_id      = rospy.get_param ("m3d_rot_frame_id", "m3d_sim/m3d_rot_laser_link" )
  subscriber1           = rospy.Subscriber("/m3d/joint_states", JointState, joinStateCallback, queue_size=1)

  rospy.spin()
