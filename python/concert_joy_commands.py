import numpy as np
from horizon.rhc.taskInterface import TaskInterface
from phase_manager import pyphase, pymanager
from sensor_msgs.msg import Joy
import rospy
import math

class GaitManager:
    def __init__(self, task_interface: TaskInterface, phase_manager: pymanager.PhaseManager, contact_map):

        # contact_map is not necessary if contact name is the same as the timeline name
        self.task_interface = task_interface
        self.phase_manager = phase_manager

        self.contact_phases = dict()

        # register each timeline of the phase manager as the contact phases
        for contact_name, phase_name in contact_map.items():
            self.contact_phases[contact_name] = self.phase_manager.getTimelines()[phase_name]

        # self.zmp_timeline = self.phase_manager.getTimelines()['zmp_timeline']

    def cycle(self, cycle_list):

        # how do I know that the stance phase is called stance_{c} or flight_{c}?
        for flag_contact, contact_name in zip(cycle_list, self.contact_phases.keys()):
            phase_i = self.contact_phases[contact_name]
            if flag_contact == 1:
                phase_i.addPhase(phase_i.getRegisteredPhase(f'stance_{contact_name}'))
            else:
                phase_i.addPhase(phase_i.getRegisteredPhase(f'flight_{contact_name}'))


    def stand(self):
        cycle_list = [1, 1, 1, 1]
        self.cycle(cycle_list)
        # self.zmp_timeline.addPhase(self.zmp_timeline.getRegisteredPhase('zmp_phase'))


class JoyCommands:
    def __init__(self, gait_manager: GaitManager):
        self.gait_manager = gait_manager
        self.base_weight = 0.5

        self.smooth_joy_msg = None
        self.joy_msg = None

        self.final_base_xy = self.gait_manager.task_interface.getTask('final_base_xy')
        self.com_height = self.gait_manager.task_interface.getTask('com_height')
        self.base_orientation = self.gait_manager.task_interface.getTask('base_orientation')

        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.publisher = rospy.Publisher('smooth_joy', Joy, queue_size=1)
        rospy.wait_for_message('/joy', Joy, timeout=0.5)

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

    def setBasePosWeight(self, w):
        self.base_weight = w
    def run(self, solution):
        if self.smooth_joy_msg is not None:
            self.smooth()

        if np.abs(self.smooth_joy_msg.axes[0]) > 0.1 or np.abs(self.smooth_joy_msg.axes[1]) > 0.1:
            # move base on x-axis in local frame
            vec = np.array([self.base_weight * self.smooth_joy_msg.axes[1],
                            self.base_weight * self.smooth_joy_msg.axes[0], 0])

            rot_vec = self._rotate_vector(vec, solution['q'][[6, 3, 4, 5], 0])
            # reference = np.array([[solution['q'][0, 0] + rot_vec[0], solution['q'][1, 0] + rot_vec[1], 0., 0., 0., 0., 0.]]).T
            reference = np.array([[1.5 * rot_vec[0], 1. * rot_vec[1]]]).T
            self.final_base_xy.setRef(reference)
        else:
            # move it back in the middle
            # reference = np.array([[solution['q'][0, 0], solution['q'][1, 0], 0., 0., 0., 0., 0.]]).T
            reference = np.array([[0., 0.]]).T
            self.final_base_xy.setRef(reference)

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

    def _rotate_vector(self, vector, quaternion):

        # normalize the quaternion
        quaternion = quaternion / np.linalg.norm(quaternion)

        # construct a pure quaternion
        v = np.array([0, vector[0], vector[1], vector[2]])

        # rotate the vector p = q* v q
        rotated_v = self._quaternion_multiply(quaternion, self._quaternion_multiply(v, self._conjugate_quaternion(quaternion)))

        # extract the rotated vector
        rotated_vector = rotated_v[1:]

        return rotated_vector

    def _quat_to_eul(self, x_quat, y_quat, z_quat, w_quat):

        # convert quaternion to Euler angles
        roll = math.atan2(2 * (w_quat * x_quat + y_quat * z_quat), 1 - 2 * (x_quat * x_quat + y_quat * y_quat))
        pitch = math.asin(2 * (w_quat * y_quat - z_quat * x_quat))
        yaw = math.atan2(2 * (w_quat * z_quat + x_quat * y_quat), 1 - 2 * (y_quat * y_quat + z_quat * z_quat))

        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return np.array([roll, pitch, yaw])
