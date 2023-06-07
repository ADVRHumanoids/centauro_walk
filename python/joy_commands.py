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

    def cycle_short(self, cycle_list):
        # how do I know that the stance phase is called stance_{c} or flight_{c}?
        for flag_contact, contact_name in zip(cycle_list, self.contact_phases.keys()):
            phase_i = self.contact_phases[contact_name]
            if flag_contact == 1:
                phase_i.addPhase(phase_i.getRegisteredPhase(f'stance_{contact_name}_short'))
            else:
                phase_i.addPhase(phase_i.getRegisteredPhase(f'flight_{contact_name}_short'))

    def cycle(self, cycle_list):
        # how do I know that the stance phase is called stance_{c} or flight_{c}?
        for flag_contact, contact_name in zip(cycle_list, self.contact_phases.keys()):
            phase_i = self.contact_phases[contact_name]
            if flag_contact == 1:
                phase_i.addPhase(phase_i.getRegisteredPhase(f'stance_{contact_name}'))
            else:
                phase_i.addPhase(phase_i.getRegisteredPhase(f'flight_{contact_name}'))


    def step(self, swing_contact):
        cycle_list = [True if contact_name != swing_contact else False for contact_name in self.contact_phases.keys()]
        self.cycle(cycle_list)


    def trot_jumped(self):

        #  diagonal 1 duration 4
        self.contact_phases['ball_2'].addPhase(self.contact_phases['ball_2'].getRegisteredPhase(f'flight_ball_2'))
        self.contact_phases['ball_3'].addPhase(self.contact_phases['ball_3'].getRegisteredPhase(f'flight_ball_3'))

        # diagonal 2 short stance 1 (3 times)
        self.contact_phases['ball_1'].addPhase(self.contact_phases['ball_1'].getRegisteredPhase(f'stance_ball_1_short'))
        self.contact_phases['ball_1'].addPhase(self.contact_phases['ball_1'].getRegisteredPhase(f'stance_ball_1_short'))
        self.contact_phases['ball_1'].addPhase(self.contact_phases['ball_1'].getRegisteredPhase(f'stance_ball_1_short'))
        self.contact_phases['ball_4'].addPhase(self.contact_phases['ball_4'].getRegisteredPhase(f'stance_ball_4_short'))
        self.contact_phases['ball_4'].addPhase(self.contact_phases['ball_4'].getRegisteredPhase(f'stance_ball_4_short'))
        self.contact_phases['ball_4'].addPhase(self.contact_phases['ball_4'].getRegisteredPhase(f'stance_ball_4_short'))

        #  diagonal 2 duration 4
        self.contact_phases['ball_1'].addPhase(self.contact_phases['ball_1'].getRegisteredPhase(f'flight_ball_1'))
        self.contact_phases['ball_4'].addPhase(self.contact_phases['ball_4'].getRegisteredPhase(f'flight_ball_4'))

        # diagonal 1 short stance 1 (3 times)
        self.contact_phases['ball_2'].addPhase(self.contact_phases['ball_2'].getRegisteredPhase(f'stance_ball_2_short'))
        self.contact_phases['ball_2'].addPhase(self.contact_phases['ball_2'].getRegisteredPhase(f'stance_ball_2_short'))
        self.contact_phases['ball_2'].addPhase(self.contact_phases['ball_2'].getRegisteredPhase(f'stance_ball_2_short'))
        self.contact_phases['ball_3'].addPhase(self.contact_phases['ball_3'].getRegisteredPhase(f'stance_ball_3_short'))
        self.contact_phases['ball_3'].addPhase(self.contact_phases['ball_3'].getRegisteredPhase(f'stance_ball_3_short'))
        self.contact_phases['ball_3'].addPhase(self.contact_phases['ball_3'].getRegisteredPhase(f'stance_ball_3_short'))


        # self.contact_phases['ball_1'].addPhase(self.contact_phases['ball_1'].getRegisteredPhase(f'stance_ball_1'))
        # self.contact_phases['ball_2'].addPhase(self.contact_phases['ball_2'].getRegisteredPhase(f'flight_ball_2'))
        # self.contact_phases['ball_3'].addPhase(self.contact_phases['ball_3'].getRegisteredPhase(f'stance_ball_3'))
        # self.contact_phases['ball_4'].addPhase(self.contact_phases['ball_4'].getRegisteredPhase(f'flight_ball_4'))



    def trot(self):
        cycle_list_1 = [0, 1, 1, 0]
        cycle_list_2 = [1, 0, 0, 1]
        self.cycle(cycle_list_1)
        self.cycle(cycle_list_2)

    def crawl(self):
        cycle_list_1 = [0, 1, 1, 1]
        cycle_list_2 = [1, 1, 1, 0]
        cycle_list_3 = [1, 0, 1, 1]
        cycle_list_4 = [1, 1, 0, 1]
        self.cycle(cycle_list_1)
        self.cycle(cycle_list_2)
        self.cycle(cycle_list_3)
        self.cycle(cycle_list_4)

    def leap(self):
        cycle_list_1 = [0, 0, 1, 1]
        cycle_list_2 = [1, 1, 0, 0]
        self.cycle(cycle_list_1)
        self.cycle(cycle_list_2)

    def walk(self):
        cycle_list_1 = [1, 0, 1, 0]
        cycle_list_2 = [0, 1, 0, 1]
        self.cycle(cycle_list_1)
        self.cycle(cycle_list_2)

    def jump(self):
        cycle_list = [0, 0, 0, 0]
        self.cycle(cycle_list)

    def wheelie(self):
        cycle_list = [0, 0, 1, 1]
        self.cycle(cycle_list)

    def stand(self):
        cycle_list = [1, 1, 1, 1]
        self.cycle(cycle_list)


class JoyCommands:
    def __init__(self, gait_manager: GaitManager):
        self.gait_manager = gait_manager
        self.base_weight = 10.
        self.base_rot_weight = 0.5
        self.com_height_w = 0.02

        self.joy_msg = None

        self.final_base_xy = self.gait_manager.task_interface.getTask('final_base_xy')
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
                # self.gait_manager.trot_jumped()
                # self.gait_manager.slide()
                # self.gait_manager.crawl()
                # self.gait_manager.leap()
                # self.gait_manager.walk()
                # self.gait_manager.jump()
                # self.gait_manager.wheelie()
                # self.gait_manager.step('wheel_1')
        else:
            # stand
            if self.gait_manager.contact_phases['ball_1'].getEmptyNodes() > 0:
                self.gait_manager.stand()

        if np.abs(self.joy_msg.axes[0]) > 0.1 or np.abs(self.joy_msg.axes[1]) > 0.1:
            # move com on x axis w.r.t the base

            vec = np.array([self.base_weight * self.joy_msg.axes[1],
                            self.base_weight * self.joy_msg.axes[0], 0])

            rot_vec = self._rotate_vector(vec, solution['q'][[6, 3, 4, 5], 0])
            reference = np.array([[solution['q'][0, 0] + rot_vec[0], solution['q'][1, 0] + rot_vec[1], 0., 0., 0., 0., 0.]]).T

            self.final_base_xy.setRef(reference)
        else:
            # move it back in the middle
            reference = np.array([[solution['q'][0, 0], solution['q'][1, 0], 0., 0., 0., 0., 0.]]).T
            self.final_base_xy.setRef(reference)

        if np.abs(self.joy_msg.axes[3]) > 0.1:
            # rotate base around z
            d_angle = np.pi / 2 * self.joy_msg.axes[3] * self.base_rot_weight
            axis = [0, 0, 1]
            q_result = self._incremental_rotate(solution['q'][[6, 3, 4, 5], 0], d_angle, axis)

            # set orientation of the quaternion
            reference = np.array([[0., 0., 0., q_result.x, q_result.y, q_result.z, q_result.w]]).T
            self.base_orientation.setRef(reference)

        elif self.joy_msg.axes[7] == 1:
            # rotate base around y
            d_angle = np.pi / 10
            axis = [0, 1, 0]
            rot_vec = self._rotate_vector(axis, solution['q'][[6, 3, 4, 5], 0])
            q_result = self._incremental_rotate(solution['q'][[6, 3, 4, 5], 0], d_angle, rot_vec)

            # set orientation of the quaternion
            reference = np.array([[0., 0., 0., q_result.x, q_result.y, q_result.z, q_result.w]]).T
            self.base_orientation.setRef(reference)

        elif self.joy_msg.axes[7] == -1:
            # rotate base around y
            d_angle = - np.pi / 10
            axis = [0, 1, 0]
            rot_vec = self._rotate_vector(axis, solution['q'][[6, 3, 4, 5], 0])
            q_result = self._incremental_rotate(solution['q'][[6, 3, 4, 5], 0], d_angle, rot_vec)

            # set orientation of the quaternion
            reference = np.array([[0., 0., 0., q_result.x, q_result.y, q_result.z, q_result.w]]).T
            self.base_orientation.setRef(reference)

        elif self.joy_msg.axes[6] == 1:
            # rotate base around x
            d_angle = np.pi / 10
            axis = [1, 0, 0]
            rot_vec = self._rotate_vector(axis, solution['q'][[6, 3, 4, 5], 0])
            q_result = self._incremental_rotate(solution['q'][[6, 3, 4, 5], 0], d_angle, rot_vec)

            # set orientation of the quaternion
            reference = np.array([[0., 0., 0., q_result.x, q_result.y, q_result.z, q_result.w]]).T
            self.base_orientation.setRef(reference)

        elif self.joy_msg.axes[6] == -1:
            # rotate base around x
            d_angle = -np.pi / 10
            axis = [1, 0, 0]
            rot_vec = self._rotate_vector(axis, solution['q'][[6, 3, 4, 5], 0])
            q_result = self._incremental_rotate(solution['q'][[6, 3, 4, 5], 0], d_angle, rot_vec)

            # set orientation of the quaternion
            reference = np.array([[0., 0., 0., q_result.x, q_result.y, q_result.z, q_result.w]]).T
            self.base_orientation.setRef(reference)

        else:
            # set rotation of the base as the current one
            reference = np.array([[0., 0., 0., solution['q'][3, 0], solution['q'][4, 0], solution['q'][5, 0], solution['q'][6, 0]]]).T
            self.base_orientation.setRef(reference)

        if self.joy_msg.buttons[0] == 1:
            # change com height
            reference = np.array([[0., 0, solution['q'][2, 0] + self.com_height_w, 0., 0., 0., 0.]]).T
            self.com_height.setRef(reference)

        if self.joy_msg.buttons[2] == 1:
            # change com height
            reference = np.array([[0., 0, solution['q'][2, 0] - self.com_height_w, 0., 0., 0., 0.]]).T
            self.com_height.setRef(reference)

    def _incremental_rotate(self, q_initial: np.quaternion, d_angle, axis) -> np.quaternion:

        # np.quaternion is [w,x,y,z]
        q_incremental = np.array([np.cos(d_angle / 2),
                                  axis[0] * np.sin(d_angle / 2),
                                  axis[1] * np.sin(d_angle / 2),
                                  axis[2] * np.sin(d_angle / 2)
                                  ])

        # normalize the quaternion
        q_incremental /= np.linalg.norm(q_incremental)

        # initial orientation of the base

        # final orientation of the base
        q_result = np.quaternion(*q_incremental) * np.quaternion(*q_initial)

        return q_result

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
