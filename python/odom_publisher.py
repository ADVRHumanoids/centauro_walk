#!/usr/bin/python3
import rospy
import tf
from cartesian_interface.pyci_all import Affine3

from geometry_msgs.msg import PoseStamped

global base_pose
global is_callback_done
def gt_pose_callback(msg):
    global base_pose
    global is_callback_done
    base_pose = Affine3([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    is_callback_done = True

rospy.init_node('odom_publisher')

is_callback_done = False

broadcaster = tf.TransformBroadcaster()

base_sub = rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if is_callback_done:
        global base_pose
        base_pose = base_pose.inverse()

        broadcaster.sendTransform(translation=base_pose.translation,
                                  rotation=base_pose.quaternion,
                                  time=rospy.Time.now(),
                                  child='odom',
                                  parent='pelvis')

    rate.sleep()
