import copy

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
        rospy.sleep(0.01)

    robot.sense()
    q_init = robot.getPositionReference()
    q_init = robot.eigenToMap(q_init)

except:
    print(colorama.Fore.RED + 'RobotInterface not created' + colorama.Fore.RESET)
    base_pose = np.array([0.03, 0., 0.962, 0., -0.029995, 0.0, 0.99955])
    # q_init = {"LHipLat": -0.0,
    #           "LHipSag": -0.363826,
    #           "LHipYaw": 0.0,
    #           "LKneePitch": 0.731245,
    #           "LAnklePitch": -0.307420,
    #           "LAnkleRoll": 0.0,
    #           "RHipLat": 0.0,
    #           "RHipSag": -0.363826,
    #           "RHipYaw": 0.0,
    #           "RKneePitch": 0.731245,
    #           "RAnklePitch": -0.307420,
    #           "RAnkleRoll": -0.0,
    #           "WaistLat": 0.0,
    #           "WaistYaw": 0.0,
    #           "LShSag": 0.959931,
    #           "LShLat": 0.007266,
    #           "LShYaw": 0.,
    #           "LElbj": -1.919862,
    #           "LForearmPlate": 0.0,
    #           "LWrj1": -0.523599,
    #           "LWrj2": -0.0,
    #           "RShSag": 0.959931,
    #           "RShLat": -0.007266,
    #           "RShYaw": 0.,
    #           "RElbj": -1.919862,
    #           "RForearmPlate": 0.0,
    #           "RWrj1": -0.523599,
    #           "RWrj2": -0.0}

    # base_pose = np.array([-0.005, 0.037, 0.965, 0.035, -0.053, 0.494, 0.867])
    q_init = {'RWrj2': 0.0,
              'RWrj1': -0.5235989689826965,
              'RShYaw': -8.742277657347586e-08,
              'RShSag': 0.9599310159683228,
              'LWrj2': 0.0,
              'LForearmPlate': -8.742277657347586e-08,
              'LShSag': 0.9599310159683228,
              'WaistYaw': 8.742277657347586e-08,
              'RElbj': -1.919862151145935,
              'LWrj1': -0.5235990881919861,
              'LHipLat': 8.742277657347586e-08,
              'LShLat': 0.0072658974677324295,
              'LHipSag': -0.3638259470462799,
              'RKneePitch': 0.7312449812889099,
              'LShYaw': -8.742277657347586e-08,
              'LHipYaw': -8.742277657347586e-08,
              'RHipLat': 8.742277657347586e-08,
              'LKneePitch': 0.7312448620796204,
              'LAnklePitch': -0.307420015335083,
              'RHipSag': -0.36382612586021423,
              'LAnkleRoll': 3.5346798199498153e-07,
              'RHipYaw': -8.742277657347586e-08,
              'RShLat': -0.007266072556376457,
              'RAnklePitch': -0.3074200451374054,
              'RForearmPlate': -8.742277657347586e-08,
              'RAnkleRoll': 3.521586791066511e-07,
              'LElbj': -1.9198620319366455,
              'WaistLat': 8.742277657347586e-08}
    base_twist = np.zeros(6)
    robot = None

'''
Initialize Horizon problem
'''
ns = 20
T = 1.7
dt = T / ns

logger = matlogger.MatLogger2('/tmp/mpc_logger')

prb = Problem(ns, receding=True, casadi_type=cs.SX)
dt_param = prb.createParameter('dt', 1)
dt_param.assign(dt)
prb.setDt(dt_param)


kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

system_order = 2
model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 sys_order_degree=system_order)

rospy.set_param('/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/cogimon_config.yaml')

com_reference = base_pose
ti.getTask('com_height').setRef(np.atleast_2d(com_reference).T)
ti.getTask('final_base_x').setRef(np.atleast_2d(base_pose).T)
# ti.getTask('final_base_y').setRef(np.atleast_2d(base_pose).T)
ti.getTask('base_orientation').setRef(np.atleast_2d(base_pose).T)

model.q.setBounds(model.kd.q_min(), model.kd.q_max())



cd_fun = ti.model.kd.computeCentroidalDynamics()

# adding minimization of angular momentum
h_lin, h_ang, dh_lin, dh_ang = cd_fun(model.q, model.v, model.a)
prb.createIntermediateResidual('min_angular_mom', 1e-1 * dh_ang)


# open loop regularizations
prb.createResidual('vel_regularization',  1. * model.v)

acc_reference = prb.createParameter('acc_reference', model.nv)
acc_reference.assign(np.zeros(model.nv))

prb.createIntermediateResidual('acc_regularization', 5e-2 * (model.a - acc_reference))


force_reference = dict()
for force_name, force_value in model.fmap.items():
    force_reference[f'force_reference_{force_name}'] = prb.createParameter(f'force_reference_{force_name}', 3)
    force_reference[f'force_reference_{force_name}'].assign(np.array([0, 0, model.kd.mass() / 8]))

    prb.createIntermediateResidual(f'force_regularization_z_{force_name}', 5e-3 * (force_value[2] - force_reference[f'force_reference_{force_name}'][2]))
    prb.createIntermediateResidual(f'force_regularization_xy_{force_name}', 5e-3 * (force_value[0:2]))

# close loop regularizations
# prb.createResidual('vel_regularization', 10. * model.v)
#
# acc_reference = prb.createParameter('acc_reference', model.nv)
# acc_reference.assign(np.zeros(model.nv))
# prb.createResidual('acc_regularization', 5e-2 * (model.a - acc_reference))
#
# force_reference = dict()
# for force_name, force_value in model.fmap.items():
#     force_reference[f'force_reference_{force_name}'] = prb.createParameter(f'force_reference_{force_name}', 3)
#     force_reference[f'force_reference_{force_name}'].assign(np.array([0, 0, model.kd.mass() / 8]))
#     prb.createResidual(f'force_regularization_{force_name}', 1e-4 * (force_value[2] - force_reference[f'force_reference_{force_name}'][2]))
#     prb.createResidual(f'force_regularization_xy_{force_name}', 5e-3 * (force_value[0:2]))
# #
# prb.createIntermediateResidual('jerk_regularization', 1e-3 * model.j)
# for force_name in model.fmap.keys():
#     prb.createIntermediateResidual(f'force_dot_regularization_{force_name}', 1e-3 * model.getInput()[f'fdot_{force_name}'])


'''
Foot vertices relative distance constraint
'''
pos_lf = model.kd.fk('l_sole')(q=model.q)['ee_pos']
pos_rf = model.kd.fk('r_sole')(q=model.q)['ee_pos']
base_ori = utils.toRot(model.kd.fk('base_link')(q=model.q0)['ee_rot'])
rel_dist = base_ori @ (pos_lf - pos_rf)

# prb.createResidual('relative_distance_lower_x', utils.barrier(rel_dist[0] + 0.3))
# prb.createResidual('relative_distance_upper_x', utils.barrier1(rel_dist[0] - 0.4))
prb.createResidual('relative_distance_lower_y', 10. * utils.barrier(rel_dist[1] - 0.25))
prb.createResidual('relative_distance_upper_y', utils.barrier1(rel_dist[1] + 1.))

active_contact_param = dict()
for frame in model.getContactFrames():
    active_contact_param[frame] = prb.createParameter(f'{frame}_contact_param', 1)

# weight_input = prb.createParameter('weight_input', 1)
# weight_input.assign(0.)
# u_input = prb.createParameter('u_param', prb.getInput().getVars().shape[0])
# prb.createResidual('min_input', weight_input * (prb.getInput().getVars() - u_input), nodes=[0])
# prb.createIntermediateResidual('min_input', weight_input * (prb.getInput().getVars() - u_input))
'''
ZMP constraint
'''


def zmp(model):
    # formulation in forces
    num = cs.SX([0, 0])
    den = cs.SX([0])

    q = cs.SX.sym('q', model.nq)
    v = cs.SX.sym('v', model.nv)
    a = cs.SX.sym('a', model.nv)

    pos_contact = dict()
    force_val = dict()

    com = model.kd.centerOfMass()(q=q, v=v, a=a)['com']

    n = cs.SX([0, 0, 1])
    for c in model.fmap.keys():
        pos_contact[c] = model.kd.fk(c)(q=q)['ee_pos']
        force_val[c] = cs.SX.sym('force_val', 3)
        num += (pos_contact[c][0:2] - com[0:2]) * cs.dot(force_val[c], n)
        den += cs.dot(force_val[c], n)

    zmp = com[0:2] + (num / den)
    input_list = []
    input_list.append(q)
    input_list.append(v)
    input_list.append(a)

    for elem in force_val.values():
        input_list.append(elem)

    f = cs.Function('zmp', input_list, [zmp])
    return f


c_mean = cs.SX([0, 0])
c_sum = cs.SX([0])
for c in model.getContactFrames():
    fk_c_pos = kin_dyn.fk(c)(q=model.q)['ee_pos']
    c_mean += fk_c_pos[0:2] * active_contact_param[c]
    c_sum += active_contact_param[c]

c_mean /= c_sum

input_zmp = list()
input_zmp.append(model.q)
input_zmp.append(model.v)
input_zmp.append(model.a)

for f_var in model.fmap.values():
    input_zmp.append(f_var)

f = zmp(model)
zmp_value = f(*input_zmp)
# prb.createResidual('zmp', 1. * (zmp_value - c_mean), nodes=range(ns))

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns + 1)
# phase manager handling
c_phases = dict()
for c in model.cmap:
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

step_time = 0.8
for c in model.cmap:
    # stance phase
    time_flight = step_time
    stance_duration = int(time_flight / dt)
    stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    stance_phase.addItem(ti.getTask(f'foot_contact_{c}'))
    stance_phase.addParameterValues(active_contact_param[c], np.ones(shape=[1, stance_duration]))
    c_phases[c].registerPhase(stance_phase)

    time_double_stance = 0.4
    short_stance_duration = int(time_double_stance / dt)
    short_stance_phase = pyphase.Phase(short_stance_duration, f'short_stance_{c}')
    short_stance_phase.addItem(ti.getTask(f'foot_contact_{c}'))
    short_stance_phase.addParameterValues(active_contact_param[c], np.ones(shape=[1, short_stance_duration]))
    c_phases[c].registerPhase(short_stance_phase)

    time_flight = step_time
    flight_duration = int(time_flight / dt)
    flight_phase = pyphase.Phase(flight_duration, f'flight_{c}')
    flight_phase.addParameterValues(active_contact_param[c], np.zeros(shape=[1, flight_duration]))
    #
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    #
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.07, [None, 0, None]))
    #
    flight_phase.addItemReference(ti.getTask(f'foot_z_{c}'), ref_trj)
    # flight_phase.addItem(ti.getTask(f'foot_contact_{c}'), nodes=[flight_duration-1]) #, nodes=[flight_duration-1]

    v_contact = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    # p_contact = model.kd.fk(c)(q=model.q)['ee_pos'].elements()
    last_swing_vel = prb.createConstraint(f'{c}last_swing_vel', v_contact, [])
    # last_swing_zero = prb.createConstraint(f'{c}_last_swing_zero', p_contact[2] - init_z_foot, [])
    flight_phase.addConstraint(last_swing_vel, nodes=[0, flight_duration - 1])
    # for contact in contact_dict[c]['vertex_frames']:
    #     flight_phase.addVariableBounds(prb.getVariables(f'f_{contact}'), np.array([[0, 0, 0]]).T, np.array([[np.inf, np.inf, np.inf]]).T, nodes=[flight_duration-1])

    c_phases[c].registerPhase(flight_phase)

for c in model.cmap:
    stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
    flight = c_phases[c].getRegisteredPhase(f'flight_{c}')
    short_stance = c_phases[c].getRegisteredPhase(f'short_stance_{c}')
    while c_phases[c].getEmptyNodes() > 0:
        c_phases[c].addPhase(stance)

model.q.setBounds(model.q0, model.q0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=ns)
# model.a.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=0)
model.q.setInitialGuess(ti.model.q0)
# model.v.setInitialGuess(ti.model.v0)
# model.a.setInitialGuess(np.zeros(model.nv))

# prb.createFinalConstraint('cazzi', model.v)
f0 = [0, 0, kin_dyn.mass() / 8 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

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
                                           model.kd_frame, model.kd, trajectory_markers=contact_list_repl)


def step(swing, stance):
    c_phases[swing].addPhase(c_phases[swing].getRegisteredPhase(f'flight_{swing}'))
    c_phases[stance].addPhase(c_phases[stance].getRegisteredPhase(f'stance_{stance}'))
    c_phases[stance].addPhase(c_phases[stance].getRegisteredPhase(f'short_stance_{stance}'))
    c_phases[swing].addPhase(c_phases[swing].getRegisteredPhase(f'short_stance_{swing}'))


global joy_msg
np.set_printoptions(precision=3, suppress=True)
import time

# from geometry_msgs.msg import PointStamped
zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)



# fig1 = plt.figure()
# ax_v = plt.subplot()
#
# fig2 = plt.figure()
# ax_f1 = plt.subplot()
# ax_f2 = plt.subplot()

# hplt = plotter.Plotter(prb, solution)

F_integrator = prb.getIntegrator()
# cost_list = ['com_height', 'joint_posture_legs', 'joint_posture_upper_body', 'base_orientation']
# cost_tasks = [ti.getTask(task_name) for task_name in cost_list]
# weight_tasks = [ti.getTask(task_name).getWeight() for task_name in cost_list]

x_init = solution['x_opt'][:, 0]
u_init = solution['u_opt'][:, 0]

# advancement_dt = dt/3
# dt_array = np.insert(dt_param.getValues(), [0], [0])[:-1]
# dt_vect = np.cumsum(dt_array)
#
# _, ax_1 = plt.subplots(1, 1)
# ax_1.grid()
# _, ax_2 = plt.subplots(1, 1)

# minmax_ax = 0.5
# ax_1.set_ylim(-minmax_ax, minmax_ax)

t = 0.
shift_num = -1

# old_solution = copy.copy(solution)
# plt.ion()
# plt.show()

# weight_input.assign(1.)
# u_input.assign(solution['u_opt'][:, 0])
time_elapsed_all_list = list()

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
        q = robot.getJointPosition()
        # q = robot.getPositionReference()
        q = np.hstack([base_pose, q])
        model.q.setBounds(q, q, nodes=0)
        qdot = robot.getJointVelocity()
        # qdot = robot.getVelocityReference()
        qdot = np.hstack([base_twist, qdot])
        model.v.setBounds(qdot, qdot, nodes=0)


    pm.shift()

    # add a new phase when the timeline starts to be empty
    for c in model.cmap:
        if c_phases[c].getEmptyNodes() > 0:
            if joy_msg.buttons[4] == 1:
                if c_phases['l_sole'].getEmptyNodes() > 0:
                    print(colorama.Fore.RED)
                    print('-=====================================================================-')
                    print('-============================= STEPPING ISSUED =======================-')
                    print('-=====================================================================-')
                    print(colorama.Fore.RESET)
                    step('l_sole', 'r_sole')
                    step('r_sole', 'l_sole')
            else:
                # for c in model.cmap:
                #     if c_phases[c].getEmptyNodes() > 0:
                c_phases[c].addPhase(c_phases[c].getRegisteredPhase(f'stance_{c}'))

    if np.abs(joy_msg.axes[1]) > 0.1:
        final_base_x = ti.getTask('final_base_x')
        reference = np.atleast_2d(np.array([solution['q'][0, 0] + 0.5 * joy_msg.axes[1], 0., 0., 0., 0., 0., 0.]))
        final_base_x.setRef(reference.T)
    else:
        final_base_x = ti.getTask('final_base_x')
        reference = np.atleast_2d(np.array([solution['q'][0, 0], 0., 0., 0., 0., 0., 0.]))
        final_base_x.setRef(reference.T)

    if np.abs(joy_msg.axes[0]) > 0.1:
        final_base_y = ti.getTask('final_base_y')
        reference = np.atleast_2d(np.array([0., solution['q'][1, 0] + 0.5 * joy_msg.axes[0], 0., 0., 0., 0., 0.]))
        final_base_y.setRef(reference.T)
    else:
        final_base_y = ti.getTask('final_base_y')
        reference = np.atleast_2d(np.array([0., solution['q'][1, 0], 0., 0., 0., 0., 0.]))
        final_base_y.setRef(reference.T)
    #
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
    toc = time.time()
    sol_time = toc - tic
    print(f'{colorama.Fore.RED}sol_time: {sol_time}{colorama.Fore.RESET}')
    t += sol_time

    # get new solution and overwrite old one
    solution = ti.solution

    new_dt_array = np.insert(dt_param.getValues(), [0], t)[:-1]
    new_dt_vect = np.cumsum(new_dt_array)


    # ax_1.grid()
    # ax_1.set_ylim(-minmax_ax, minmax_ax)

    # for dim in range(solution['u_opt'].shape[0]):
    # for dim in range(1):
    #     ax_1.step(dt_vect[:-1], old_solution['u_opt'][dim, :], color='blue', where='post')
    #     ax_1.step(new_dt_vect[:-1], solution['u_opt'][dim, :], color='red', where='post')
    #     ax_1.scatter(dt_vect[:-1], old_solution['u_opt'][dim, :])
    #     ax_1.scatter(new_dt_vect[:-1], solution['u_opt'][dim, :], marker='x')
    #
    # for dim in range(solution['u_opt'].shape[0]):
    # for dim in range(1):
    #     ax_2.plot(dt_vect, old_solution['x_opt'][dim, :], color='blue')
    #     ax_2.plot(new_dt_vect, new_solution['x_opt'][dim, :], color='red', linestyle='--')

    for force_name in model.getForceMap():
        logger.add(force_name, solution[f'f_{force_name}'][:, 0])

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
            Vector3(x=solution[f'f_{frame}'][0, 1], y=solution[f'f_{frame}'][1, 1], z=solution[f'f_{frame}'][2, 1]))
        # print(f"forces of frame: {frame}\n {solution[f'f_{frame}'][2, 1]}")
        # sol_msg.fdot.append(Vector3(x=solution[f'fdot_{frame}'][0, 1], y=solution[f'fdot_{frame}'][1, 1], z=solution[f'fdot_{frame}'][2, 1]))

    solution_publisher.publish(sol_msg)

    # replay stuff

    if robot is None:
        repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
        repl.publish_joints(solution['q'][:, 0])
        repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)

    time_elapsed_all = time.time() - tic
    time_elapsed_all_list.append(time_elapsed_all)

    rate.sleep()

    # print(f"{colorama.Style.RED}MPC loop elapsed time: {time.time() - tic}{colorama.Style.RESET}")

    # print(f'average time elapsed shifting: {sum(time_elapsed_shifting_list) / len(time_elapsed_shifting_list)}')
    # print(f'average time elapsed solving: {sum(time_elapsed_solving_list) / len(time_elapsed_solving_list)}')
    print(f'average time elapsed solving: {sum(time_elapsed_all_list) / len(time_elapsed_all_list)}')






















