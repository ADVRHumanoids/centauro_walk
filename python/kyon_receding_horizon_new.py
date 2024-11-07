#!/usr/bin/python3

import horizon.utils.kin_dyn as kd
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from horizon.ros import replay_trajectory
from horizon.utils.resampler_trajectory import Resampler
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
from horizon.rhc.ros.task_server_class import TaskServerClass
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
from geometry_msgs.msg import PointStamped
import cartesian_interface.roscpp_utils as roscpp
import horizon.utils.analyzer as analyzer

from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS
import convex_plane_decomposition_ros.pysegmented_plane_projection as projector
from rosbot_param_server import rosbot_param_server_py

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from scipy.spatial.transform import Rotation
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

def set_base_state_from_robot():
    # numerical problem: two quaternions can represent the same rotation
    # if difference between the base orientation in the state x and the sensed one base_pose < 0, change sign
    state_quat_conjugate = np.copy(x_opt[3:7, 0])
    state_quat_conjugate[:3] *= -1.0

    # normalize the quaternion
    state_quat_conjugate = state_quat_conjugate / np.linalg.norm(x_opt[3:7, 0])
    diff_quat = _quaternion_multiply(base_pose[3:], state_quat_conjugate)

    if diff_quat[3] < 0:
        base_pose[3:] = -base_pose[3:]

    model.q[0:7].setBounds(base_pose, base_pose, nodes=0)

    # VELOCITY OF PINOCCHIO IS LOCAL, BASE_TWIST FROM  XBOTCORE IS GLOBAL:
    # transform it in local
    r_base = Rotation.from_quat(base_pose[3:]).as_matrix()

    r_adj = np.zeros([6, 6])
    r_adj[:3, :3] = r_base.T
    r_adj[3:6, 3:6] = r_base.T

    # rotate in the base frame the relative velocity (ee_v_distal - ee_v_base_distal)
    ee_rel = r_adj @ base_twist
    model.v[0:6].setBounds(ee_rel, ee_rel, nodes=0)


def set_state_from_robot(robot_joint_names, q_robot, qdot_robot, fixed_joint_map={}):
    robot.sense()

    # manage fixed joints if any
    q_map = robot.getJointPositionMap()

    for fixed_joint in fixed_joint_map:
        if fixed_joint in q_map:
            del q_map[fixed_joint]

    q_index = 0
    for j_name in robot_joint_names:
        q_robot[q_index] = q_map[j_name]
        q_index += 1

    # numerical problem: two quaternions can represent the same rotation
    # if difference between the base orientation in the state x and the sensed one base_pose < 0, change sign
    state_quat_conjugate = np.copy(x_opt[3:7, 0])
    state_quat_conjugate[:3] *= -1.0

    # normalize the quaternion
    state_quat_conjugate = state_quat_conjugate / np.linalg.norm(x_opt[3:7, 0])
    diff_quat = _quaternion_multiply(base_pose[3:], state_quat_conjugate)

    if diff_quat[3] < 0:
        base_pose[3:] = -base_pose[3:]

    q = np.hstack([base_pose, q_robot])
    model.q.setBounds(q, q, nodes=0)

    qdot = robot.getJointVelocity()
    qdot_map = robot.eigenToMap(qdot)

    for fixed_joint in fixed_joint_map:
        if fixed_joint in qdot_map:
            del qdot_map[fixed_joint]

    qdot_index = 0
    for j_name in robot_joint_names:
        qdot_robot[qdot_index] = qdot_map[j_name]
        qdot_index += 1

    # VELOCITY OF PINOCCHIO IS LOCAL, BASE_TWIST FROM  XBOTCORE IS GLOBAL:
    # transform it in local
    r_base = Rotation.from_quat(base_pose[3:]).as_matrix()

    r_adj = np.zeros([6, 6])
    r_adj[:3, :3] = r_base.T
    r_adj[3:6, 3:6] = r_base.T

    # rotate in the base frame the relative velocity (ee_v_distal - ee_v_base_distal)
    ee_rel = r_adj @ base_twist

    qdot = np.hstack([ee_rel, qdot_robot])
    model.v.setBounds(qdot, qdot, nodes=0)

rospy.init_node('kyon_walk_srbd')
projector.init('kyon_control_node', [])


solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)

rospy.sleep(1.)

'''
Load urdf and srdf
'''
urdf = rospy.get_param(param_name='/robot_description', default='')
if urdf == '':
    raise print('urdf not set')

srdf = rospy.get_param(param_name='/robot_description_semantic', default='')
if srdf == '':
    raise print('srdf not set')

file_dir = os.getcwd()

'''
Build ModelInterface and RobotStatePublisher
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

'''
Force Estimation
'''
model_xbot = xbot.ModelInterface(cfg)
# f_est = pyest.ForceEstimation(model_xbot, 1e-3)
# force_sensor_dict = dict()
# ees = ['contact_1', 'contact_2', 'contact_3', 'contact_4']
# chains = ['front_left_leg', 'front_right_leg', 'rear_left_leg', 'rear_right_leg']
# for ee, chain in zip(ees, chains):
#     force_sensor_dict[ee] = f_est.addLink(ee, [0, 1, 2], [chain])

'''
params
'''
closed_loop = rospy.get_param(param_name='~closed_loop', default=False)
xbot_param = rospy.get_param(param_name="~xbot", default=False)
perception = rospy.get_param(param_name="~perception", default=False)
joystick_flag = rospy.get_param(param_name='~joy', default=False)

'''
Initialize Horizon problem
'''
ns = 30
T = 1.5
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

'''
Build ModelInterface and RobotStatePublisher
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

base_pose = None
base_twist = None
robot = None

try:
    robot = xbot.RobotInterface(cfg)
    rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)
    rospy.Subscriber('/xbotcore/link_state/pelvis/twist', TwistStamped, gt_twist_callback)
    while base_pose is None or base_twist is None:
        rospy.sleep(0.01)
    robot.sense()
    q_init = robot.getJointPosition()
    q_init = robot.eigenToMap(q_init)

except:
    print('RobotInterface not created')
    q_init = {'hip_roll_1': 0.0,
              'hip_pitch_1': 0.7,
              'knee_pitch_1': -1.4,
              'hip_roll_2': 0.0,
              'hip_pitch_2': -0.7,
              'knee_pitch_2': 1.4,
              'hip_roll_3': 0.0,
              'hip_pitch_3': 0.7,
              'knee_pitch_3': -1.4,
              'hip_roll_4': 0.0,
              'hip_pitch_4': -0.7,
              'knee_pitch_4': 1.4,
              # 'wheel_joint_1': 0.0,
              # 'wheel_joint_2': 0.0,
              # 'wheel_joint_3': 0.0,
              # 'wheel_joint_4': 0.0,
              # 'shoulder_yaw_1': 0.0,
              # 'shoulder_pitch_1': 0.9,
              # 'elbow_pitch_1': 1.68,
              # 'wrist_pitch_1': 0.,
              # 'wrist_yaw_1': 0.,
              # 'shoulder_yaw_2': 0.0,
              # 'shoulder_pitch_2': 0.9,
              # 'elbow_pitch_2': 1.68,
              # 'wrist_pitch_2': 0.,
              # 'wrist_yaw_2': 0.
              }

    base_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    FK = kin_dyn.fk('contact_1')
    init = base_pose.tolist() + list(q_init.values())
    init_pos_foot = FK(q=kin_dyn.mapToQ(q_init))['ee_pos']
    base_pose[2] = -init_pos_foot[2]

    if 'wheel_joint_1' in kin_dyn.joint_names() and robot is not None:
        base_pose[2] += 0.124

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose
                                 )

rospy.set_param('mpc/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=mpc/robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)



if 'wheel_joint_1' in model.kd.joint_names():
    ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/wheel_config.yaml')
else:
    ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/kyon_config_new.yaml')

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_timelines = dict()
for c in model.cmap.keys():
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')

# weight more roll joints
white_list_indices = list()
black_list_indices = list()
white_list = ['hip_roll_1', 'hip_roll_2', 'hip_roll_3', 'hip_roll_4']

if 'wheel_joint_1' in model.kd.joint_names():
    black_list = ['wheel_joint_1', 'wheel_joint_2', 'wheel_joint_3', 'wheel_joint_4']
else:
    black_list = []

postural_joints = np.array(list(range(7, model.nq)))
for joint in black_list:
    black_list_indices.append(model.joint_names.index(joint))
for joint in white_list:
    white_list_indices.append(7 + model.joint_names.index(joint))
postural_joints = np.delete(postural_joints, black_list_indices)

min_q_white_list_weight = prb.createParameter('min_q_white_list_weight', 1)
min_q_white_list_weight.assign(5.)
if white_list:
    prb.createResidual("min_q_white_list", min_q_white_list_weight * (model.q[white_list_indices] - model.q0[white_list_indices]))

short_stance_duration = 1
stance_duration = 10
flight_duration = 10

FK_contacts = dict()
dFK_contacts = dict()
for c in model.getContactMap():
    FK_contacts[c] = model.kd.fk(c)
    dFK_contacts[c] = model.kd.frameVelocity(c, model.kd_frame)

for c in model.cmap.keys():
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
    if ti.getTask(f'{c}_contact') is not None:
        stance_phase.addItem(ti.getTask(f'{c}_contact'))
        stance_phase_short.addItem(ti.getTask(f'{c}_contact'))
    else:
        raise Exception('task not found')

    # c_timelines[c].registerPhase(stance_phase)
    # c_timelines[c].registerPhase(stance_phase_short)

    # flight phase normal
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ee_vel = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.1, [None, 0, None]))
    if ti.getTask(f'z_{c}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
    else:
        raise Exception('task not found')
    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

    ref_trj_xy = np.zeros(shape=[7, 1])
    ref_trj_xy[0:2, 0] = FK_contacts[c](q=model.q0)['ee_pos'].elements()[0:2]
    flight_phase.addItemReference(ti.getTask(f'xy_{c}'), ref_trj_xy, nodes=[flight_duration - 1])

for c in model.cmap.keys():
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

vel_lims = model.kd.velocityLimits()
prb.createResidual('max_vel', 1e1 * utils.utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))

test = prb.createParameter('test', 2)
test.assign(np.atleast_2d([3, 3]).T)

# finalize taskInterface and solve bootstrap problem
ti.finalize()



tsc = TaskServerClass(ti)
tsc.addParameter('min_q_white_list_weight', min_q_white_list_weight)
tsc.setMinMax('min_q_white_list_weight', 0, 100)
tsc.setMinMax('joint_posture_weight', 0, 10)
tsc.setMinMax("acceleration_regularization_weight", 0, 0.1)
tsc.setMinMax("velocity_regularization_weight", 0, 2.)
for cname in model.getContactMap().keys():
    tsc.setMinMax(f"{cname}_regularization_weight", 0, 0.1)

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)

global joy_msg

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()
time_elapsed_solving_list = list()
time_elapsed_all_list = list()

contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

# if joystick_flag:
from centauro_joy_commands import JoyCommands
jc = JoyCommands()

gait_manager_ros = GaitManagerROS(gm)

if 'wheel_joint_1' in model.kd.joint_names():
    gait_manager_ros.setBaseRotWeight(0.1)
    gait_manager_ros.setBaseRotWeight(0.1)
else:
    gait_manager_ros.setBasePoseWeight(1.5)
    gait_manager_ros.setBaseRotWeight(1.)

# if 'wheel_joint_1' in model.kd.joint_names():
#     from geometry_msgs.msg import PointStamped
#     zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)

# anal = analyzer.ProblemAnalyzer(prb)

# import matplotlib.pyplot as plt
# plt.ion()  # Turn on interactive mode
# fig, ax = plt.subplots()
# line, = ax.plot(range(prb.getNNodes() - 1), ti.solver_bs.getConstraintsValues()['dynamics'][0, :])  # Plot initial data
# ax.set_ylim(-2., 2.)  # Set your desired limits here

def _quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])

robot_joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

q_robot = np.zeros(len(robot_joint_names))
qdot_robot = np.zeros(len(robot_joint_names))

while not rospy.is_shutdown():
    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]

    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    # closed loop
    if closed_loop:
        set_state_from_robot(robot_joint_names=robot_joint_names, q_robot=q_robot, qdot_robot=qdot_robot)

    # perception
    if jc.perception:
        project = False
        set_base_state_from_robot()
        for c, timeline in c_timelines.items():
            for phase in timeline.getActivePhases():
                if phase.getName() == f'flight_{c}':
                    final_node = phase.getPosition() + phase.getNNodes()
                    if final_node < ns + 1:

                        initial_pose = FK_contacts[c](q=solution['q'][:, phase.getPosition()])['ee_pos'].elements()
                        projected_initial_pose = projector.project(initial_pose)
                        landing_pose = FK_contacts[c](q=solution['q'][:, final_node])['ee_pos'].elements()
                        projected_final_pose = projector.project(landing_pose)

                        query_point = PointStamped()
                        query_point.header.frame_id = 'odom'
                        query_point.header.stamp = rospy.Time.now()
                        query_point.point.x = landing_pose[0]
                        query_point.point.y = landing_pose[1]
                        query_point.point.z = landing_pose[2]

                        projected_point = PointStamped()
                        projected_point.header = query_point.header
                        projected_point.point.x = projected_final_pose[0]
                        projected_point.point.y = projected_final_pose[1]
                        projected_point.point.z = projected_final_pose[2]

                        qp = np.array([query_point.point.x, query_point.point.y, query_point.point.z])
                        pp = np.array([projected_point.point.x, projected_point.point.y, projected_point.point.z])
                        if 0.02 < np.linalg.norm(qp - pp) < 0.1:
                            ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration,
                                                                              projected_initial_pose[2],
                                                                              projected_final_pose[2],
                                                                              0.1,
                                                                              [None, 0, None]))

                            phase.setItemReference(f'z_{c}', ref_trj)

                            phase.setItemWeight(f'xy_{c}', [50.])
                            ref_trj_xy[0:2, 0] = projected_final_pose[0:2]
                            phase.setItemReference(f'xy_{c}', ref_trj_xy)

    # shift phases of phase manager
    tic = time.time()
    pm.shift()
    time_elapsed_shifting = time.time() - tic
    time_elapsed_shifting_list.append(time_elapsed_shifting)

    jc.run()
    gait_manager_ros.run()

    tic = time.time()
    ti.rti()
    time_elapsed_solving = time.time() - tic
    time_elapsed_solving_list.append(time_elapsed_solving)

    solution = ti.solution

    sol_msg = WBTrajectory()
    sol_msg.header.frame_id = 'world'
    sol_msg.header.stamp = rospy.Time.now()

    sol_msg.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

    sol_msg.q = solution['q'][:, 0].tolist()
    sol_msg.v = solution['v'][:, 0].tolist()
    sol_msg.a = solution['a'][:, 0].tolist()

    for frame in model.getForceMap():
        sol_msg.force_names.append(frame)
        sol_msg.f.append(
            Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))

    solution_publisher.publish(sol_msg)

    # replay stuff
    if robot is None:
        repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
        repl.publish_joints(solution['q'][:, 0])
        # repl.publish_joints(solution['q'][:, ns], prefix='last')
        repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
        # repl.publish_future_trajectory_marker('base_link', solution['q'][0:3, :])
        # repl.publish_future_trajectory_marker('contact1', solution['q'][8:11, :])

    time_elapsed_all = time.time() - tic
    time_elapsed_all_list.append(time_elapsed_all)

    tsc.update()
    rate.sleep()


projector.shutdown()