#!/usr/bin/python3

from cartesian_interface.pyci_all import *
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, analyzer, utils
from horizon.transcriptions import integrators
from horizon.ros import replay_trajectory
from horizon.utils import plotter
import matplotlib.pyplot as plt
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline

from sensor_msgs.msg import Joy
from kyon_controller.msg import WBTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PointStamped

from matplotlib import pyplot as hplt

from matlogger2 import matlogger

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import colorama


global joy_msg
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


def joy_callback(msg):
    global joy_msg
    joy_msg = msg

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def conjugate_quaternion(q):
    q_conjugate = np.copy(q)
    q_conjugate[1:] *= -1.0
    return q_conjugate
def rotate_vector(vector, quaternion):
    # normalize the quaternion
    quaternion = quaternion / np.linalg.norm(quaternion)

    # construct a pure quaternion
    v = np.array([0, vector[0], vector[1], vector[2]])

    # rotate the vector p = q* v q
    rotated_v = quaternion_multiply(quaternion,
                                          quaternion_multiply(v, conjugate_quaternion(quaternion)))

    # extract the rotated vector
    rotated_vector = rotated_v[1:]

    return rotated_vector

def incremental_rotate(q_initial: np.quaternion, d_angle, axis) -> np.quaternion:
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


rospy.init_node('cogimon_walk_srbd')

solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=10)

rospy.Subscriber('/joy', Joy, joy_callback)

'''
Load urdf and srdf
'''
cogimon_urdf_folder = rospkg.RosPack().get_path('cogimon_urdf')
cogimon_srdf_folder = rospkg.RosPack().get_path('cogimon_srdf')

urdf = open(cogimon_urdf_folder + '/urdf/cogimon.urdf', 'r').read()
srdf = open(cogimon_srdf_folder + '/srdf/cogimon.srdf', 'r').read()

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

base_pose = None
base_twist = None
try:
    robot = xbot.RobotInterface(cfg)
    # rospy.Subscriber('/cogimon_base_estimation/base_link/pose', PoseStamped, gt_pose_callback)
    # rospy.Subscriber('/cogimon_base_estimation/base_link/twist', TwistStamped, gt_twist_callback)
    rospy.Subscriber('/xbotcore/link_state/base_link/pose', PoseStamped, gt_pose_callback)
    rospy.Subscriber('/xbotcore/link_state/base_link/twist', TwistStamped, gt_twist_callback)

    while base_pose is None or base_twist is None:
        print('trying')
        rospy.sleep(0.01)

    base_pose = np.array([0.03, 0., 0.962, 0., -0.029995, 0.0, 0.99955])
    base_twist = np.zeros(6)

    robot.sense()
    q_init = robot.getPositionReference()
    q_init = robot.eigenToMap(q_init)
    print(colorama.Fore.CYAN + 'RobotInterface created!' + colorama.Fore.RESET)

except:
    print(colorama.Fore.CYAN + 'RobotInterface not created' + colorama.Fore.RESET)
    base_pose = np.array([0.03, 0., 0.962, 0., -0.029995, 0.0, 0.99955])

    q_init = {"WaistLat": 0.0,
              # "WaistYaw": 0.0,
              "RShSag": 0.959931,
              "RShLat": -0.007266,
              "RShYaw": -0.0,
              "RElbj": -1.919862,
              "RForearmPlate": 0.0,
              "RWrj1": -0.523599,
              "LShSag": 0.959931,
              "LShLat": 0.007266,
              "LShYaw": -0.0,
              "LElbj": -1.919862,
              "LForearmPlate": 0.0,
              "LWrj1": -0.523599,
              "RHipLat": 0.0,
              "RHipSag": -0.363826,
              "RHipYaw": 0.0,
              "RKneePitch": 0.731245,
              "RAnklePitch": -0.307420,
              "RAnkleRoll": -0.0,
              "LHipLat": -0.0,
              "LHipSag": -0.363826,
              "LHipYaw": 0.0,
              "LKneePitch": 0.731245,
              "LAnklePitch": -0.307420,
              "LAnkleRoll": 0.0}

    base_twist = np.zeros(6)
    robot = None

'''
Initialize Horizon problem
'''
ns = 30
T = 1.5
dt = T / ns

logger = matlogger.MatLogger2('/tmp/mpc_logger')

prb = Problem(ns, receding=True, casadi_type=cs.SX)
dt_param = prb.createParameter('dt', 1)
dt_param.assign(dt)
prb.setDt(dt_param)

fixed_joint_map = {'WaistYaw': 0.0}
kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joint_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 fixed_joint_map=fixed_joint_map)

rospy.set_param('/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/cogimon_config.yaml')

cd_fun = ti.model.kd.computeCentroidalDynamics()

# adding minimization of angular momentum
h_lin, h_ang, dh_lin, dh_ang = cd_fun(model.q, model.v, model.a)
prb.createIntermediateResidual('min_angular_mom', 1e-1 * dh_ang)


'''
Foot vertices relative distance constraint
'''
pos_lf = model.kd.fk('l_sole')(q=model.q)['ee_pos']
pos_rf = model.kd.fk('r_sole')(q=model.q)['ee_pos']
base_ori = model.kd.fk('base_link')(q=model.q)['ee_rot']
rel_dist = base_ori.T @ (pos_lf - pos_rf)

# prb.createResidual('relative_distance_lower_x', utils.barrier(rel_dist[0] + 0.3))
# prb.createResidual('relative_distance_upper_x', utils.barrier1(rel_dist[0] - 0.4))
prb.createResidual('relative_distance_lower_y', 10. * utils.barrier(rel_dist[1] - 0.21))
# prb.createResidual('relative_distance_upper_y', 10. * utils.barrier1(rel_dist[1] - 0.35))

force_z_ref = dict()
for contact, force in model.getForceMap().items():
    print(f'{contact}: {force}')
    force_z_ref[contact] = prb.createParameter(f'{contact}_force_z_ref', 1)
    prb.createIntermediateResidual(f'{contact}_force_z_reg', 0.001 * (force[2] - force_z_ref[contact]))

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns + 1)
# phase manager handling
c_timelines = dict()
for c in model.cmap:
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')

step_time = 0.7
for c in model.cmap:
    # stance phase
    time_flight = step_time
    stance_duration = int(time_flight / dt)
    stance_duration = 15
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    stance_phase.addItem(ti.getTask(f'foot_contact_{c}'))

    time_double_stance = 0.4
    short_stance_duration = int(time_double_stance / dt)
    short_stance_duration = 7
    short_stance_phase = c_timelines[c].createPhase(short_stance_duration, f'short_stance_{c}')
    short_stance_phase.addItem(ti.getTask(f'foot_contact_{c}'))

    time_flight = step_time
    flight_duration = int(time_flight / dt)
    flight_duration = 15
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.05, [None, 0, None]))
    flight_phase.addItemReference(ti.getTask(f'foot_z_{c}'), ref_trj)
    # flight_phase.addItem(ti.getTask(f'zero_vel_xy_{c}'))

for c in model.cmap:
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    flight = c_timelines[c].getRegisteredPhase(f'flight_{c}')
    short_stance = c_timelines[c].getRegisteredPhase(f'short_stance_{c}')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)

model.q.setBounds(model.q0, model.q0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=ns)
model.q.setInitialGuess(ti.model.q0)

# prb.createFinalConstraint('cazzi', model.v)
f0 = [0, 0, kin_dyn.mass() / 8 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

for contact, force in model.getForceMap().items():
    force_z_ref[contact].assign(f0[2])

# finalize taskInterface and solve bootstrap problem
ti.finalize()
ti.bootstrap()

solution = ti.solution

iteration = 0
rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd, trajectory_markers=contact_list_repl,
                                           fixed_joint_map=fixed_joint_map)


def step(swing, stance):
    c_timelines[swing].addPhase(c_timelines[swing].getRegisteredPhase(f'flight_{swing}'))
    c_timelines[stance].addPhase(c_timelines[stance].getRegisteredPhase(f'stance_{stance}'))
    c_timelines[stance].addPhase(c_timelines[stance].getRegisteredPhase(f'short_stance_{stance}'))
    c_timelines[swing].addPhase(c_timelines[swing].getRegisteredPhase(f'short_stance_{swing}'))


global joy_msg
import time

while not rospy.is_shutdown():

    tic = time.time()
    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]

    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    if robot is not None:
        robot.sense()
        q = robot.getJointPositionMap()
        for fixed_joint in fixed_joint_map.keys():
            del q[fixed_joint]
        # q = np.hstack([base_pose, q])
        model.q[7:].setBounds(list(q.values()), list(q.values()), nodes=0)
        qdot = robot.getJointVelocityMap()
        for fixed_joint in fixed_joint_map.keys():
            del qdot[fixed_joint]
        # qdot = np.hstack([base_twist, qdot])
        model.v[6:].setBounds(list(qdot.values()), list(qdot.values()), nodes=0)


    pm.shift()

    # add a new phase when the timeline starts to be empty
    for c in model.cmap:
        if c_timelines[c].getEmptyNodes() > 0:
            if joy_msg.buttons[4] == 1:
                if c_timelines['l_sole'].getEmptyNodes() > 0:
                    step('l_sole', 'r_sole')
                    step('r_sole', 'l_sole')
            else:
                c_timelines[c].addPhase(c_timelines[c].getRegisteredPhase(f'stance_{c}'))

    if np.abs(joy_msg.axes[1]) > 0.1 or np.abs(joy_msg.axes[0]) > 0.1:
        final_base_xy = ti.getTask('final_base_xy')
        vec = np.array([joy_msg.axes[1], joy_msg.axes[0], 0])
        rot_vec = rotate_vector(vec, solution['q'][[6, 3, 4, 5], 0])
        print(rot_vec)
        reference = np.atleast_2d(np.array([solution['q'][0, 0] + 1.5 * rot_vec[0], solution['q'][1, 0] + 1.5 * rot_vec[1], 0., 0., 0., 0., 0.]))
        final_base_xy.setRef(reference.T)

    else:
        final_base_xy = ti.getTask('final_base_xy')
        reference = np.atleast_2d(np.array([solution['q'][0, 0], solution['q'][1, 0], 0., 0., 0., 0., 0.]))
        final_base_xy.setRef(reference.T)

    if np.abs(joy_msg.axes[3]) > 0.1:
        base_orientation_task = ti.getTask('base_yaw_orientation')
        base_reference_yaw = np.array([[0., 0., 0., 0, 0, 0, 0]]).T
        d_angle = np.pi / 2 * 0.25 * joy_msg.axes[3]
        axis = [0, 0, 1]
        angular_velocity_vector = incremental_rotate(solution['q'][[6, 3, 4, 5], 0], d_angle, axis)
        base_reference_yaw[3] = angular_velocity_vector.x
        base_reference_yaw[4] = angular_velocity_vector.y
        base_reference_yaw[5] = angular_velocity_vector.z
        base_reference_yaw[6] = angular_velocity_vector.w
        base_orientation_task.setRef(base_reference_yaw)
    else:
        base_orientation_task = ti.getTask('base_yaw_orientation')
        base_reference_yaw = np.atleast_2d(np.array([0., 0., 0., solution['q'][3, 0], solution['q'][4, 0], solution['q'][5, 0], solution['q'][6, 0]]))
        base_orientation_task.setRef(base_reference_yaw.T)

    if joy_msg.buttons[3] == 1:
        com_height = ti.getTask('com_height')
        reference = np.atleast_2d(np.array([0., 0., solution['q'][2, 0] + 0.05, 0., 0., 0., 0.]))
        com_height.setRef(reference.T)
    elif joy_msg.buttons[2] == 1:
        com_height = ti.getTask('com_height')
        reference = np.atleast_2d(np.array([0., 0., solution['q'][2, 0] - 0.05, 0., 0., 0., 0.]))
        com_height.setRef(reference.T)

    # solve real time iteration
    ti.rti()
    # toc = time.time()
    # sol_time = toc - tic
    # print(f'{colorama.Fore.RED}sol_time: {sol_time}{colorama.Fore.RESET}')

    # get new solution and overwrite old one
    solution = ti.solution

    for force_name in model.getForceMap():
        logger.add(force_name, solution[f'f_{force_name}'][:, 0])

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
        repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)

    # time_elapsed_all = time.time() - tic

    rate.sleep()






















