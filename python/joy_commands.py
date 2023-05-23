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
            # move com on x axis w.r.t the base
            vec = np.array([solution['q'][0, 0] + self.base_weight * self.joy_msg.axes[1], 0, 0])
            rot_vec = self.rotate_vector(vec, solution['q'][[3, 4, 5, 6], 0])

            print(vec)
            print(rot_vec)
            reference = np.atleast_2d(
                np.array([rot_vec[0], rot_vec[1], rot_vec[2], 0., 0., 0., 0.]))
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

    def _quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])

    def _conjugate_quaternion(self, q):
        q_conjugate = np.copy(q)
        q_conjugate[1:] *= -1.0
        return q_conjugate

    def rotate_vector(self, vector, quaternion):

        # normalize the quaternion
        quaternion = quaternion / np.linalg.norm(quaternion)

        # construct a pure quaternion
        v = np.array([0, vector[0], vector[1], vector[2]])

        # rotate the vector
        rotated_v = self._quaternion_multiply(quaternion, self._quaternion_multiply(v, self._conjugate_quaternion(quaternion)))

        # extract the rotated vector
        rotated_vector = rotated_v[1:]

        return rotated_vector

