from typing import Dict
import numpy as np
from geometry_msgs.msg import WrenchStamped
from functools import partial
import rospy
import colorama

class ContactDetectorROS:
    def __init__(self, contact_dict: Dict):

        # contact_dict = contact_name-topic
        self._contact_dict = contact_dict

        # contact_dict = contact_name-wrench
        self._wrench_dict = dict()
        for key in self._contact_dict:
            self._wrench_dict[key] = None

        self._init_subscribers()

    def _contact_callback(self, msg, contact_name):

        self._wrench_dict[contact_name] = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                                    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

    def _init_subscribers(self):

        for contact_name, topic in self._contact_dict.items():
            rospy.Subscriber(topic, WrenchStamped, self._contact_callback, callback_args=contact_name)
            rospy.wait_for_message(topic, WrenchStamped, timeout=5)


    def getWrench(self, contact):

        return self._wrench_dict[contact]

    def getContact(self, contact):

        threshold = 20.

        if np.linalg.norm(self._wrench_dict[contact][:3]) > threshold:
            return True

        return False


    def run(self):

        for contact_name, wrench in self._wrench_dict.items():

            if self.getContact(contact_name):
                print(f"{contact_name}: {colorama.Fore.GREEN}DETECTED{colorama.Style.RESET_ALL}")
            else:
                print(f"{contact_name}: {colorama.Fore.RED}NON DETECTED{colorama.Style.RESET_ALL}")


if __name__ == '__main__':

    rospy.init_node('contact_detector_node')

    c_dict = dict(ball_1='/ft_sensors/knee_pitch_1',
                  ball_2='/ft_sensors/knee_pitch_2',
                  ball_3='/ft_sensors/knee_pitch_3',
                  ball_4='/ft_sensors/knee_pitch_4')

    cdROS = ContactDetectorROS(c_dict)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        cdROS.run()
        rate.sleep()

