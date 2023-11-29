#!/usr/bin/python3
import horizon.utils.kin_dyn as kd
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils
from horizon.ros import replay_trajectory
from horizon.utils.resampler_trajectory import Resampler
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
from matlogger2 import matlogger
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
from sensor_msgs.msg import Joy
import cartesian_interface.roscpp_utils as roscpp
import horizon.utils.analyzer as analyzer
import colorama
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from kyon_controller.msg import WBTrajectory

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import time


import horizon.utils as utils

global base_pose
global base_twist

def gt_pose_callback(msg):
    global base_pose
    base_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                          msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                          msg.pose.orientation.w])


def gt_twist_callback(msg):
    global base_twist
    base_twist = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                           msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])


rospy.init_node('kyon_walk_srbd')
roscpp.init('kyon_walk_srbd', [])

'''
Load urdf and srdf
'''


urdf_urdf_folder = rospkg.RosPack().get_path('kyon_urdf')
srdf_srdf_folder = rospkg.RosPack().get_path('kyon_srdf')
urdf = open(urdf_urdf_folder + '/urdf/kyon.urdf', 'r').read()
srdf = open(srdf_srdf_folder + '/srdf/kyon.srdf', 'r').read()

file_dir = os.getcwd()

'''
Initialize Horizon problem
'''
ns = 40
T = 3
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

'''
Build ModelInterface and RobotStatePublisher
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

base_pose = None
base_twist = None

try:
    robot = xbot.RobotInterface(cfg)
    rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)
    rospy.Subscriber('/xbotcore/link_state/pelvis/twist', TwistStamped, gt_twist_callback)
    while base_pose is None or base_twist is None:
        rospy.sleep(0.01)
    robot.sense()
    q_init = robot.getPositionReference()
    q_init = robot.eigenToMap(q_init)

except:
    print('RobotInterface not created')
    q_init = {'hip_roll_1': 0.0,
              'hip_pitch_1': 0.7,
              'knee_pitch_1': -1.4,
              'hip_roll_2': 0.0,
              'hip_pitch_2': 0.7,
              'knee_pitch_2': -1.4,
              'hip_roll_3': 0.0,
              'hip_pitch_3': 0.7,
              'knee_pitch_3': -1.4,
              'hip_roll_4': 0.0,
              'hip_pitch_4': 0.7,
              'knee_pitch_4': -1.4,
              'wheel_joint_1': 0.0,
              'wheel_joint_2': 0.0,
              'wheel_joint_3': 0.0,
              'wheel_joint_4': 0.0,
              'shoulder_yaw_1': 0.0,
              'shoulder_pitch_1': 0.9,
              'elbow_pitch_1': 1.68,
              'wrist_pitch_1': 0.,
              'wrist_yaw_1': 0.,
              'shoulder_yaw_2': 0.0,
              'shoulder_pitch_2': 0.9,
              'elbow_pitch_2': 1.68,
              'wrist_pitch_2': 0.,
              'wrist_yaw_2': 0.}

base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

FK = kin_dyn.fk('ball_1')
init = base_init.tolist() + list(q_init.values())
init_pos_foot = FK(q=kin_dyn.mapToQ(q_init))['ee_pos']
base_init[2] = -init_pos_foot[2]

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init
                                 )

rospy.set_param('robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)

ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/contact_config.yaml')

# ========================= set up contact detection =========================
c_dict = dict(ball_1='/ft_sensors/knee_pitch_1',
              ball_2='/ft_sensors/knee_pitch_2',
              ball_3='/ft_sensors/knee_pitch_3',
              ball_4='/ft_sensors/knee_pitch_4')

from contact_detection_ros import ContactDetectorROS
cdROS = ContactDetectorROS(c_dict)

flag_contacts = dict()
for f_name in model.getForceMap().keys():
    flag_contacts[f_name] = prb.createSingleParameter(f'contact_flag_{f_name}', 1)
    flag_contacts[f_name].assign(1)
# ============================================================================
com_height = ti.getTask('com_height')
com_height.setRef(np.atleast_2d(base_init).T)

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_phases = dict()
for c in model.cmap.keys():
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

stance_duration = 5
flight_duration = 5
for c in model.cmap.keys():
    # stance phase normal
    stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    if ti.getTask(f'{c}_contact') is not None:
        stance_phase.addItem(ti.getTask(f'{c}_contact'))
    else:
        raise Exception('task not found')

    c_phases[c].registerPhase(stance_phase)

    # flight phase normal
    flight_phase = pyphase.Phase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.05, [None, 0, None]))
    if ti.getTask(f'z_{c}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
    else:
        raise Exception('task not found')
    # flight_phase.addConstraint(prb.getConstraints(f'{c}_vert'), nodes=[0 ,flight_duration-1])  # nodes=[0, 1, 2]
    c_phases[c].registerPhase(flight_phase)

'''
Maximize support polygon
'''
# pos1 = model.kd.fk('ball_1')(q=model.q)['ee_pos']
# pos2 = model.kd.fk('ball_2')(q=model.q)['ee_pos']
# pos3 = model.kd.fk('ball_3')(q=model.q)['ee_pos']
# pos4 = model.kd.fk('ball_4')(q=model.q)['ee_pos']
#
# sp_area = cs.fabs(((pos1[0] * pos2[1]) + (pos2[0] * pos3[1]) + (pos3[0] * pos4[1]) + (pos4[0] * pos1[1])) -
#                   ((pos1[1] * pos2[0]) + (pos2[1] * pos3[0]) + (pos3[1] * pos4[0]) + (pos4[1] * pos1[0]))) / 2
#
# prb.createResidual('max_support_polygon', 1e-1 * (sp_area - 3.))

for c in model.cmap.keys():
    stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
    while c_phases[c].getEmptyNodes() > 0:
        c_phases[c].addPhase(stance)

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
# ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
# ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

vel_lims = model.kd.velocityLimits()
prb.createResidual('max_vel', 1e1 * utils.utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))
# finalize taskInterface and solve bootstrap problem
ti.finalize()

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

iteration = 0
rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)
                                           # future_trajectory_markers={'base_link': 'world', 'ball_1': 'world'})

global joy_msg

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()

# from key_commands import GaitManager, KeyCommands
from joy_commands import GaitManager, JoyCommands

contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

# jc = JoyCommands(gm)
# kc = KeyCommands(gm)


from geometry_msgs.msg import PointStamped
zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)
zmp_f = ti.getTask('zmp')._zmp_fun()

all_contact_flag = True
while not rospy.is_shutdown():
    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]

    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    # shift phases of phase manager
    tic = time.time()
    pm.shift()
    time_elapsed_shifting = time.time() - tic
    time_elapsed_shifting_list.append(time_elapsed_shifting)

    # jc.run(solution)
    # kc.run()

    iteration = iteration + 1

    ti.rti()
    solution = ti.solution

    sol_msg = WBTrajectory()
    sol_msg.header.frame_id = 'world'
    sol_msg.header.stamp = rospy.Time.now()

    sol_msg.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

    sol_msg.q = solution['q'][:, 1].tolist()
    sol_msg.v = solution['v'][:, 1].tolist()
    sol_msg.a = solution['a'][:, 1].tolist()

    for frame in model.getForceMap():
        sol_msg.force_names.append(frame)
        sol_msg.f.append(
            Vector3(x=solution[f'f_{frame}'][0, 1], y=solution[f'f_{frame}'][1, 1], z=solution[f'f_{frame}'][2, 1]))



    for contact_name in model.getContacts():
        if cdROS.getContact(contact_name):
            flag_contacts[contact_name].assign(1.)
            print(f"{contact_name}: {colorama.Fore.GREEN}DETECTED{colorama.Style.RESET_ALL}")
        else:
            flag_contacts[contact_name].assign(0.)
            print(f"{contact_name}: {colorama.Fore.RED}NON DETECTED{colorama.Style.RESET_ALL}")

    all_contact_flag = all(elem.getValues()[0][0] == 1. for elem in flag_contacts.values())

    if all_contact_flag:
        # step
        if gm.contact_phases['ball_1'].getEmptyNodes() > 0:
            print('adding stand')
            gm.stand()
    else:
        # stand
        if gm.contact_phases['ball_1'].getEmptyNodes() > 0:
            print('adding trot')
            gm.trot()

    # replay stuff
    # =========================== publish zmp =================================================
    # input_zmp = []
    # input_zmp.append(solution['q'][:, 0])
    # input_zmp.append(solution['v'][:, 0])
    # input_zmp.append(solution['a'][:, 0])
    #
    # for f_var in model.fmap.keys():
    #     input_zmp.append(solution[f"f_{f_var}"][:, 0])
    #
    # c_mean = np.zeros([3, 1])
    # for c_name, f_var in model.fmap.items():
    #     fk_c_pos = kin_dyn.fk(c_name)(q=solution['q'][:, 0])['ee_pos'].toarray()
    #     c_mean += fk_c_pos
    #
    # c_mean /= len(model.cmap.keys())
    #
    # zmp_val = zmp_f(*input_zmp)
    #
    # zmp_point = PointStamped()
    # zmp_point.header.stamp = rospy.Time.now()
    # zmp_point.header.frame_id = 'world'
    # zmp_point.point.x = zmp_val[0]
    # zmp_point.point.y = zmp_val[1]
    # zmp_point.point.z = 0
    #
    # zmp_pub.publish(zmp_point)
    # ============================================================================
    repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
    repl.publish_joints(solution['q'][:, 0])
    repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
    # repl.publish_future_trajectory_marker('base_link', solution['q'][0:3, :])
    # repl.publish_future_trajectory_marker('ball_1', solution['q'][8:11, :])
    rate.sleep()

print(f'average time elapsed shifting: {sum(time_elapsed_shifting_list) / len(time_elapsed_shifting_list)}')
