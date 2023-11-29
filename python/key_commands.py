import numpy as np
from horizon.rhc.taskInterface import TaskInterface
from phase_manager import pyphase, pymanager
# from key_controller.msg import Key
from std_msgs.msg import String
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
        # self.zmp_timeline.addPhase(self.zmp_timeline.getRegisteredPhase('zmp_empty_phase'))
        # self.zmp_timeline.addPhase(self.zmp_timeline.getRegisteredPhase('zmp_empty_phase'))

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
        # self.zmp_timeline.addPhase(self.zmp_timeline.getRegisteredPhase('zmp_phase'))


class KeyCommands:
    def __init__(self, gait_manager: GaitManager):
        self.key_msg = String()
        self.gait_manager = gait_manager
        self.base_weight = 1.
        self.base_rot_weight = 0.5
        self.com_height_w = 0.02

        rospy.Subscriber('/keyboard_input', String, self.key_callback)

        self._init_tasks()
        # rospy.wait_for_message('/keyboard_input', String, timeout=0.5)
    def _init_tasks(self):

        self.final_base_xy = self.gait_manager.task_interface.getTask('final_base_xy')

    def key_callback(self, msg: String):
        self.key_msg = msg

    def run(self):

        if self.key_msg.data == 'Key.ctrl Key.enter':
            # step
            if self.gait_manager.contact_phases['ball_1'].getEmptyNodes() > 0:
                self.gait_manager.trot()
        else:
            # stand
            if self.gait_manager.contact_phases['ball_1'].getEmptyNodes() > 0:
                self.gait_manager.stand()


        if self.key_msg.data == 'Key.ctrl Key.up':

            reference = np.array([[1., 1.]]).T

            self.final_base_xy.setRef(reference)
        else:
            # move it back in the middle
            # reference = np.array([[solution['q'][0, 0], solution['q'][1, 0], 0., 0., 0., 0., 0.]]).T
            reference = np.array([[0., 0.]]).T
            self.final_base_xy.setRef(reference)