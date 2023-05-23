import numpy as np
from horizon.rhc.taskInterface import TaskInterface
from phase_manager import pyphase, pymanager
from sensor_msgs.msg import Joy
import rospy

class GaitManager:
    def __init__(self, task_interface: TaskInterface, phase_manager: pymanager.PhaseManager, contact_map):

        # contact_map is not necessary if contact name is the same as the timeline name
        self.task_interface = task_interface
        self.phase_manager = phase_manager

        self.contact_phases = dict()

        for contact_name, phase_name in contact_map.items():
            self.contact_phases[contact_name] = self.phase_manager.getTimelines()[phase_name]


    def cycle(self, cycle_list):
        for flag_contact, contact_name in zip(cycle_list, self.contact_phases.keys()):
            if flag_contact == 1:
                self.contact_phases[contact_name].addPhase(self.contact_phases[contact_name].getRegisteredPhase(f'stance_{contact_name}'))
            else:
                self.contact_phases[contact_name].addPhase(self.contact_phases[contact_name].getRegisteredPhase(f'flight_{contact_name}'))


    def step(self, swing_contact):
        cycle_list = [True if contact_name != swing_contact else False for contact_name in self.contact_phases.keys()]
        self.cycle(cycle_list)


    def trot(self):
        cycle_list_1 = [0, 1, 1, 0]
        cycle_list_2 = [1, 0, 0, 1]
        self.cycle(cycle_list_1)
        self.cycle(cycle_list_2)

    def stand(self):
        # how do I know that the stance phase is called (f'stance_{c}')?
        for c in self.contact_phases:
            if self.contact_phases[c].getEmptyNodes() > 0:
                self.contact_phases[c].addPhase(self.contact_phases[c].getRegisteredPhase(f'stance_{c}'))


class JoyCommands:
    def __init__(self, gait_manager: GaitManager):
        self.gait_manager = gait_manager
        self.base_weight = 0.5
        self.base_rot_weight = 1.
        self.com_height_w = 0.02

        self.joy_msg = None

        self.final_base_x = self.gait_manager.task_interface.getTask('final_base_x')
        self.final_base_y = self.gait_manager.task_interface.getTask('final_base_y')
        self.com_height = self.gait_manager.task_interface.getTask('com_height')
        self.base_orientation = self.gait_manager.task_interface.getTask('base_orientation')

        rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.wait_for_message('/joy', Joy, timeout=0.5)

    def joy_callback(self, msg):
        self.joy_msg = msg

    def run(self, solution):

        if self.joy_msg.buttons[4] == 1:
            # step
            if self.gait_manager.contact_phases['ball_1'].getEmptyNodes() > 0:
                self.gait_manager.trot()
        else:
            # stand
            self.gait_manager.stand()

        if np.abs(self.joy_msg.axes[1]) > 0.1:
            # move com on x axis
            reference = np.atleast_2d(
                np.array([solution['q'][0, 0] + self.base_weight * self.joy_msg.axes[1], 0., 0., 0., 0., 0., 0.]))
            self.final_base_x.setRef(reference.T)
        else:
            # move it back in the middle
            reference = np.atleast_2d(np.array([solution['q'][0, 0], 0., 0., 0., 0., 0., 0.]))
            self.final_base_x.setRef(reference.T)

        if np.abs(self.joy_msg.axes[0]) > 0.1:
            # move com on y axis
            reference = np.atleast_2d(
                np.array([0., solution['q'][1, 0] + self.base_weight * self.joy_msg.axes[0], 0., 0., 0., 0., 0.]))
            self.final_base_y.setRef(reference.T)
        else:
            # move com back
            reference = np.atleast_2d(np.array([0., solution['q'][1, 0], 0., 0., 0., 0., 0.]))
            self.final_base_y.setRef(reference.T)

        if np.abs(self.joy_msg.axes[3]) > 0.1:
            # rotate base
            angle = np.pi / 50 * self.joy_msg.axes[3] * self.base_rot_weight
            axis = np.array([0, 0, 1])

            # np.quaternion is [w,x,y,z]
            q_incremental = np.array([np.cos(angle / 2),
                                      axis[0] * np.sin(angle / 2),
                                      axis[1] * np.sin(angle / 2),
                                      axis[2] * np.sin(angle / 2)
                                      ])

            # normalize the quaternion
            q_incremental /= np.linalg.norm(q_incremental)

            # initial orientation of the base
            q_initial = solution['q'][[6, 3, 4, 5], 0]

            # final orientation of the base
            q_result = np.quaternion(*q_incremental) * np.quaternion(*q_initial)

            # set orientation of the quaternion
            reference = np.atleast_2d(np.array([0., 0., 0., q_result.x, q_result.y, q_result.z, q_result.w]))
            self.base_orientation.setRef(reference.T)
        else:
            # set rotation of the base as the current one
            reference = np.atleast_2d(np.array([0., 0., 0., solution['q'][3, 0], solution['q'][4, 0], solution['q'][5, 0], solution['q'][6, 0]]))
            self.base_orientation.setRef(reference.T)

        if np.abs(self.joy_msg.buttons[0]) == 1:
            # change com height
            reference = np.atleast_2d(np.array([0., 0, solution['q'][2, 0] + self.com_height_w, 0., 0., 0., 0.]))
            self.com_height.setRef(reference.T)

        if np.abs(self.joy_msg.buttons[2]) == 1:
            # change com height
            reference = np.atleast_2d(np.array([0., 0, solution['q'][2, 0] - self.com_height_w, 0., 0., 0., 0.]))
            self.com_height.setRef(reference.T)