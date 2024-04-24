#!/usr/bin/python3

import horizon.utils.kin_dyn as kd
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from horizon.ros import replay_trajectory
from horizon.utils.resampler_trajectory import Resampler
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
import phase_manager.pyrosserver as pyrosserver

from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS

import cartesian_interface.roscpp_utils as roscpp
import horizon.utils.analyzer as analyzer

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from scipy.spatial.transform import Rotation
import colorama
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PolygonStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from kyon_controller.msg import WBTrajectory

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import time

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

def imu_callback(msg: Imu):
    global base_pose
    base_pose = np.zeros(7)
    base_pose[3:] = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])


# def create_stance_phase(name, duration, task_interface, contact_list):
#
#     # required task: {c}_contact
#     stance_phases = dict()
#     for contact_name in contact_list:
#         contact_task_name = f'{contact_name}_contact'
#         if task_interface.getTask(contact_task_name) is None:
#             raise Exception(f'task "{contact_task_name}" not found')
#
#         contact_task = task_interface.getTask(contact_task_name)
#         stance_phase = pyphase.Phase(duration, f'{name}_{contact_name}')
#         stance_phase.addItem(contact_task)
#
#         stance_phases[contact_name] = stance_phase
#
#         return stance_phases


# def create_stance_phase(name, duration, task_interface, contact_list):
#
#     model = task_interface.getModel()
#
#     flight_phases = dict()
#     for contact_name in contact_list:
#         z_task_name = f'z_{contact_name}'
#         if ti.getTask(z_task_name) is None:
#             raise Exception(f'task "{z_task_name}" not found')
#
#         flight_phase = pyphase.Phase(flight_duration, f'flight_{c}')
#
#         init_z_foot = fk_dict[c](q=model.q0)['ee_pos'].elements()[2]
#         ee_vel = fk_vel_dict[c](q=model.q, qdot=model.v)['ee_vel_linear']
#         ref_trj = np.zeros(shape=[7, flight_duration])
#         ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.1, [None, 0, None]))
#         if ti.getTask(f'z_{c}') is not None:
#             flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
#         else:
#             raise Exception('task not found')
#         cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
#         flight_phase.addConstraint(cstr, nodes=[0, flight_duration - 1])
#         c_timelines[c].registerPhase(flight_phase)


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
    diff_quat = utils.quaternion_multiply(base_pose[3:], state_quat_conjugate)

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



'''
load params
'''


rospy.init_node('kyon_walk')
roscpp.init('kyon_walk', [])


joystick_flag = rospy.get_param(param_name='~joy', default=False)
closed_loop = rospy.get_param(param_name='~closed_loop', default=False)
xbot_param = rospy.get_param(param_name="~xbot", default=False)

solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)
solution_time_publisher = rospy.Publisher('/mpc_solution_time', Float64, queue_size=1, tcp_nodelay=True)

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

file_dir = rospkg.RosPack().get_path('kyon_controller')

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

if xbot_param:
    robot = xbot.RobotInterface(cfg)
    robot.sense()

    if closed_loop:
        print('controller running in CLOSED LOOP')
    else:
        print('controller running in OPEN LOOP')

        # rospy.Subscriber('/xbotcore/imu/imu_link', Imu, imu_callback)
        # while base_pose is None:
        #     rospy.sleep(0.01)
        # base_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        # base_twist = np.zeros(6)
    # else:
    rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)
    rospy.Subscriber('/xbotcore/link_state/pelvis/twist', TwistStamped, gt_twist_callback)
    
    while base_pose is None or base_twist is None:
        rospy.sleep(0.01)
    # q_init = robot.getPositionReference()
    q_init = robot.getJointPosition()
    q_init = robot.eigenToMap(q_init)

else:
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

rospy.set_param('mpc/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=mpc/robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)

if 'wheel_joint_1' in model.kd.joint_names():
    ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/kyon_wheel_config.yaml')
else:
    ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/kyon_feet_config.yaml')

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_timelines = dict()
for c in model.cmap.keys():
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')

# weight more roll joints
white_list_indices = list()
black_list_indices = list()
white_list = []

if 'wheel_joint_1' in model.kd.joint_names():
    black_list = ['wheel_joint_1', 'wheel_joint_2', 'wheel_joint_3', 'wheel_joint_4']
else:
    black_list = []

# white_list = ['hip_roll_1', 'hip_roll_2', 'hip_roll_3', 'hip_roll_4']

postural_joints = np.array(list(range(7, model.nq)))
for joint in black_list:
    black_list_indices.append(model.joint_names.index(joint))
for joint in white_list:
    white_list_indices.append(7 + model.joint_names.index(joint))
postural_joints = np.delete(postural_joints, black_list_indices)

if white_list:
    prb.createResidual("min_q_white_list", 5. * (model.q[white_list_indices] - model.q0[white_list_indices]))
# if black_list:
#     prb.createResidual('velocity_regularization', 0.1 * model.v[postural_joints])

fk_dict = dict()
fk_vel_dict = dict()

for c in model.cmap.keys():
    fk_dict[c] = model.kd.fk(c)
    fk_vel_dict[c] = model.kd.frameVelocity(c, model.kd_frame)

short_stance_duration = 1
stance_duration = 8
flight_duration = 8
stance_phase_recovery_duration = 4
flight_phase_recovery_duration = 4

for c in model.cmap.keys():
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
    stance_phase_recovery = c_timelines[c].createPhase(stance_phase_recovery_duration, f'stance_{c}_recovery')

    if ti.getTask(f'{c}_contact') is not None:
        stance_phase.addItem(ti.getTask(f'{c}_contact'))
        stance_phase_short.addItem(ti.getTask(f'{c}_contact'))
        stance_phase_recovery.addItem(ti.getTask(f'{c}_contact'))
    else:
        raise Exception('task not found')

    # flight phase normal
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}')
    flight_phase_recovery = c_timelines[c].createPhase(flight_phase_recovery_duration, f'flight_{c}_recovery')

    init_z_foot = fk_dict[c](q=model.q0)['ee_pos'].elements()[2]
    ee_vel = fk_vel_dict[c](q=model.q, qdot=model.v)['ee_vel_linear']

    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.2, [None, 0, None]))

    ref_trj_recovery = np.zeros(shape=[7, flight_phase_recovery_duration])
    ref_trj_recovery[2, :] = np.atleast_2d(tg.from_derivatives(flight_phase_recovery_duration, init_z_foot, init_z_foot, 0.2, [None, 0, None]))

    if ti.getTask(f'z_{c}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
        flight_phase_recovery.addItemReference(ti.getTask(f'z_{c}'), ref_trj_recovery)
    else:
        raise Exception('task not found')

    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])
    # flight_phase_recovery.addConstraint(cstr, nodes=[0, flight_phase_recovery_duration-1])

# pos_lf = model.kd.fk('l_sole')(q=model.q)['ee_pos']
# pos_rf = model.kd.fk('r_sole')(q=model.q)['ee_pos']
# base_ori = horizon.utils.toRot(model.kd.fk('base_link')(q=model.q0)['ee_rot'])
# rel_dist = base_ori.T @ (pos_lf - pos_rf)

# prb.createResidual('relative_distance_lower_x', horizon.utils.barrier(rel_dist[0] + 0.3))
# prb.createResidual('relative_distance_upper_x', horizon.utils.barrier1(rel_dist[0] - 0.4))
# prb.createResidual('relative_distance_lower_y', 10 * horizon.utils.barrier(rel_dist[1] - 0.2))
# prb.createResidual('relative_distance_upper_y', horizon.utils.barrier1(rel_dist[1] + 1.))

# for f_name, f in model.fmap.items():
#     f_prev = f.getVarOffset(-1)
#     prb.createIntermediateResidual(f'{f_name}_smooth_forces', 1e-2 * (f_prev - f), nodes=range(1, ns-1))


def zmp_cs_fun(model):

    # formulation in forces
    num = cs.SX([0, 0])
    den = cs.SX([0])
    pos_contact = dict()
    force_val = dict()

    q = cs.SX.sym('q', model.nq)
    v = cs.SX.sym('v', model.nv)
    a = cs.SX.sym('a', model.nv)

    com = model.kd.centerOfMass()(q=q, v=v, a=a)['com']

    n = cs.SX([0, 0, 1])
    for c in model.fmap.keys():
        pos_contact[c] = fk_dict[c](q=q)['ee_pos']
        force_val[c] = cs.SX.sym('force_val', 3)
        num += (pos_contact[c][0:2] - com[0:2]) * cs.dot(force_val[c], n)
        den += cs.dot(force_val[c], n)

    zmp = com[0:2] + (num / den)
    input_list = [q, v, a]

    for elem in force_val.values():
        input_list.append(elem)

    f = cs.Function('zmp', input_list, [zmp])

    return f

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
    # stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    stance_short = c_timelines[c].getRegisteredPhase(f'stance_{c}_short')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance_short)

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
prb.createResidual('max_vel', 1e1 * utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))

# finalize taskInterface and solve bootstrap problem
ti.finalize()

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl,
                                           future_trajectory_markers=['ball_1', 'ball_2', 'ball_3', 'ball_4', 'pelvis'])

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()
time_elapsed_solving_list = list()
time_elapsed_all_list = list()

rs = pyrosserver.RosServerClass(pm)

contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

if joystick_flag:
    from joy_commands import JoyCommands
    jc = JoyCommands()

# remap names from kyon_feet_config to standard names for gait manager
gm_opts = dict()
gm_opts['task_name'] = dict(base_pose_xy='final_base_xy', base_pose_z='com_height')
gait_manager_ros = GaitManagerROS(gm, gm_opts)

# if 'wheel_joint_1' in model.kd.joint_names():

from geometry_msgs.msg import PointStamped
zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10, tcp_nodelay=True)
zmp_point = PointStamped()
zmp_fun = zmp_cs_fun(model)


# support_polygon_pub = rospy.Publisher('support_polygon', PolygonStamped, queue_size=1, tcp_nodelay=True)
support_polygon_pub = rospy.Publisher('support_polygon', MarkerArray, queue_size=1, tcp_nodelay=True)
# support_polygon_msg = PolygonStamped()
support_polygon_msg = MarkerArray()
# support_polygon_msg.frame_id = 'world'

robot_joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

q_robot = np.zeros(len(robot_joint_names))
qdot_robot = np.zeros(len(robot_joint_names))

fk_c_pos = dict()
support_polygon_vertex_list = ['ball_1', 'ball_2', 'ball_4', 'ball_3']

def distance_point_to_line(point, line_vertices):
    # Calculate the distance between a point (x, y) and a line defined by two points (x1, y1) and (x2, y2)
    distance = np.linalg.norm(np.cross(line_vertices[1] - line_vertices[0], line_vertices[0] - point)) / np.linalg.norm(line_vertices[1] - line_vertices[0])

    return distance


threshold_capture_stepping = 0.2
capture_stepping_i = 0
trot_side_flag = 1

anal = analyzer.ProblemAnalyzer(prb)

while not rospy.is_shutdown():
    # tic = time.time()
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

    # shift phases of phase manager
    tic = time.time()
    pm.shift()
    time_elapsed_shifting = time.time() - tic
    time_elapsed_shifting_list.append(time_elapsed_shifting)

    rs.run()
    if joystick_flag:
        # receive msgs from joystick and publishes to ROS topic
        jc.run()

    # receive msgs from ros topic and send commands to robot
    gait_manager_ros.run()

    # anal.printConstraints('zero_velocity_ball_1_ball_1_vel_cartesian_task', suppress_bounds=1)
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
    # sol_msg.q = solution['q'][:, 1].tolist()
    # sol_msg.v = solution['v'][:, 1].tolist()
    sol_msg.a = solution['a'][:, 0].tolist()

    for frame in model.getForceMap():
        sol_msg.force_names.append(frame)
        sol_msg.f.append(
            Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))

    solution_publisher.publish(sol_msg)

    # =========================== compute zmp =================================================
    input_zmp = []
    input_zmp.append(solution['q'][:, 0])
    input_zmp.append(solution['v'][:, 0])
    input_zmp.append(solution['a'][:, 0])
    #
    for f_var in model.fmap.keys():
        input_zmp.append(solution[f"f_{f_var}"][:, 0])
    #
    c_mean = np.zeros([3, 1])
    f_tot = np.zeros([3, 1])

    for c in model.cmap.keys():
        # position of the ee contact
        fk_c_pos[c] = fk_dict[c](q=solution['q'][:, 0])['ee_pos'].toarray()
        # position of the ee contact if in contact
        c_mean += fk_c_pos[c] * solution[f"f_{c}"][2, 0]
        f_tot += solution[f"f_{c}"][2, 0]

    c_mean /= f_tot

    zmp_val = zmp_fun(*input_zmp)

    # ================================ compute support polygon ==============================
    support_polygon_vertices = []
    for c_name in support_polygon_vertex_list:
        if c_timelines[c_name].getActivePhases()[0].getName() != f'flight_{c_name}':
            fk_c_pos[c_name] = fk_dict[c_name](q=solution['q'][:, 0])['ee_pos'][:2].toarray()

            support_polygon_vertices.append(fk_c_pos[c_name].T)

    # ================================ compute distances ==============================

    distances_support_polygon = []
    for vertex_i in range(len(support_polygon_vertices)):
        line_points = [support_polygon_vertices[vertex_i % len(support_polygon_vertices)],
                       support_polygon_vertices[(vertex_i + 1) % len(support_polygon_vertices)]]

        distances_support_polygon.append(distance_point_to_line(zmp_val.T, line_points))

    color_sp = ColorRGBA(0.0, 1.0, 0.0, 1.0)

    if len(support_polygon_vertices) == 4:
        for distance_zmp in distances_support_polygon:
            if distance_zmp < threshold_capture_stepping:
                capture_stepping_i += 1
                color_sp = ColorRGBA(1.0, 0.0, 0.0, 1.0)


    zmp_point.header.stamp = rospy.Time.now()
    zmp_point.header.frame_id = 'world'
    zmp_point.point.x = zmp_val[0]
    zmp_point.point.y = zmp_val[1]
    zmp_point.point.z = 0

    zmp_pub.publish(zmp_point)

    # support_polygon_msg.polygon.points.clear()
    support_polygon_msg.markers.clear()

    marker = Marker(type=Marker.LINE_STRIP,
                    action=Marker.ADD,
                    scale=Vector3(x=0.005),
                    color=color_sp,
                    header=Header(frame_id='world', stamp=rospy.Time.now()),)

    marker.pose.orientation.w = 1

    for vertex_i in range(len(support_polygon_vertices) + 1):
        point = Point()
        point.x = support_polygon_vertices[vertex_i % len(support_polygon_vertices)][0, 0]
        point.y = support_polygon_vertices[vertex_i % len(support_polygon_vertices)][0, 1]
        point.z = 0

        marker.points.append(point)


        support_polygon_msg.markers.append(marker)

    support_polygon_pub.publish(support_polygon_msg)


    if capture_stepping_i > 10:
        capture_stepping_i = 0



        # entering step correction (close to present horizon)
        gm.diagonal_pair_recovery(trot_side_flag, phase_pos=1)

        trot_side_flag = 1 - trot_side_flag

    # print('================================================')
    #
    # for name, timeline in c_timelines.items():
    #     print(colorama.Fore.RED + colorama.Style.BRIGHT + f'timeline: {name}'+ colorama.Style.NORMAL)
    #     for phase in timeline.getPhases():
    #         print(colorama.Fore.CYAN + f"   phase: {phase.getName()}" + colorama.Fore.RESET)
    #         print(colorama.Fore.MAGENTA + f"      real pos: {phase.getPosition() - (phase.getNNodes() - len(phase.getActiveNodes()))}" + colorama.Fore.RESET)
    #         print(colorama.Fore.MAGENTA + f"      duration: {phase.getNNodes()}" + colorama.Fore.RESET)
    #
    # print('================================================')




    # =========================== c mean =================================================

    # c_mean_point.header.stamp = rospy.Time.now()
    # c_mean_point.header.frame_id = 'world'
    # c_mean_point.point.x = c_mean[0]
    # c_mean_point.point.y = c_mean[1]
    # c_mean_point.point.z = 0
    #
    # c_mean_pub.publish(c_mean_point)
    # ============================================================================
    # replay stuff
    repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
    repl.publish_joints(solution['q'][:, 0]) # , prefix='mpc'
    # repl.publish_joints(solution['q'][:, ns], prefix='last')
    repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
    repl.publish_future_trajectory_marker(solution['q'])

    time_elapsed_all = time.time() - tic
    time_elapsed_all_list.append(time_elapsed_all)

    rate.sleep()

        # print(f"{colorama.Style.RED}MPC loop elapsed time: {time.time() - tic}{colorama.Style.RESET}")

    # print(f'average time elapsed shifting: {sum(time_elapsed_shifting_list) / len(time_elapsed_shifting_list)}')
    # print(f'average time elapsed solving: {sum(time_elapsed_solving_list) / len(time_elapsed_solving_list)}')
    # print(f'average time elapsed all: {sum(time_elapsed_all_list) / len(time_elapsed_all_list)}')

