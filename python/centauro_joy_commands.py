import numpy as np
from horizon.rhc.taskInterface import TaskInterface
from phase_manager import pyphase, pymanager
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Joy
import rospy
import math
from visualization_msgs.msg import Marker
from horizon.rhc.gait_manager import GaitManager
from geometry_msgs.msg import Twist
import time

class JoyCommands:
    def __init__(self):
        # self.base_weight = 0.5
        # self.base_rot_weight = 0.5
        # self.com_height_w = 0.1

        self.smooth_joy_msg = None
        self.joy_msg = None

        self.__base_lin_vel_weight = 1.
        self.__base_ori_vel_weight = 1.

        self.velocity_ref = Twist()
        self.perception = False
        self.__latest_click = time.time()

        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.publisher = rospy.Publisher('smooth_joy', Joy, queue_size=1)
        rospy.wait_for_message('/joy', Joy, timeout=0.5)

        self.__pub = rospy.Publisher('pos_ref', Marker, queue_size=1)

        self.__open_subscribers()

    def __open_subscribers(self):

        self.__base_vel_pub = rospy.Publisher('/horizon/base_velocity/reference', Twist)
        self.__crawl_cli = rospy.ServiceProxy('/horizon/crawl/switch', SetBool)
        self.__trot_cli = rospy.ServiceProxy('/horizon/trot/switch', SetBool)


    def setBaseVelLinWeight(self, weight):
        self.__base_lin_vel_weight = weight

    def setBaseVelOriWeight(self, weight):
        self.__base_ori_vel_weight = weight

    def smooth(self):
        alpha = 0.1
        as_list = list(self.smooth_joy_msg.axes)
        for index in range(len(self.joy_msg.axes)):
            as_list[index] = alpha * self.joy_msg.axes[index] + (1 - alpha) * self.smooth_joy_msg.axes[index]
        self.smooth_joy_msg.axes = tuple(as_list)
        self.publisher.publish(self.smooth_joy_msg)

    def joy_callback(self, msg:Joy):
        if self.smooth_joy_msg is None:
            self.smooth_joy_msg = msg
        self.joy_msg = msg

    def run(self):

        if self.smooth_joy_msg is not None:
            self.smooth()

        if self.joy_msg.buttons[4] == 1:
            self.__crawl_cli(True)
        else:
            self.__crawl_cli(False)

        if self.joy_msg.buttons[5] == 1:
            self.__trot_cli(True)
        else:
            self.__trot_cli(False)


        if np.abs(self.smooth_joy_msg.axes[0]) > 0.1 or np.abs(self.smooth_joy_msg.axes[1]) > 0.1:

            self.velocity_ref.linear.x = self.__base_lin_vel_weight * self.smooth_joy_msg.axes[1]
            self.velocity_ref.linear.y = self.__base_lin_vel_weight * self.smooth_joy_msg.axes[0]

        if np.abs(self.smooth_joy_msg.axes[3]) > 0.1:

            self.velocity_ref.angular.z = self.__base_ori_vel_weight * self.smooth_joy_msg.axes[3]

        if self.joy_msg.buttons[0] == 1:
            # change com height
            self.velocity_ref.linear.z = 0.01

        if self.joy_msg.buttons[2] == 1:
            # change com height
            self.velocity_ref.linear.z = -0.01

        if self.joy_msg.buttons[0] == 0 and self.joy_msg.buttons[2] == 0:
            self.velocity_ref.linear.z = 0.

        if self.joy_msg.axes[7] == 1:
            click_time = time.time()
            if click_time - self.__latest_click > 1.:
                self.perception = not self.perception
                self.__latest_click = click_time

        self.__base_vel_pub.publish(self.velocity_ref)




    # def _quat_to_eul(self, x_quat, y_quat, z_quat, w_quat):
    #
    #     # convert quaternion to Euler angles
    #     roll = math.atan2(2 * (w_quat * x_quat + y_quat * z_quat), 1 - 2 * (x_quat * x_quat + y_quat * y_quat))
    #     pitch = math.asin(2 * (w_quat * y_quat - z_quat * x_quat))
    #     yaw = math.atan2(2 * (w_quat * z_quat + x_quat * y_quat), 1 - 2 * (y_quat * y_quat + z_quat * z_quat))
    #
    #     roll = math.degrees(roll)
    #     pitch = math.degrees(pitch)
    #     yaw = math.degrees(yaw)
    #
    #     return np.array([roll, pitch, yaw])
