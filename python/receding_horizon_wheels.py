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

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import time

import horizon.utils as utils


rospy.init_node('kyon_walk_srbd')
roscpp.init('kyon_walk_srbd', [])

solution_publisher = rospy.Publisher('/mpc_solution', JointTrajectory, queue_size=10)
rospy.sleep(1.)

'''
Load urdf and srdf
'''

kyon_urdf_folder = rospkg.RosPack().get_path('kyon_urdf')
kyon_srdf_folder = rospkg.RosPack().get_path('kyon_srdf')


urdf = subprocess.check_output(["xacro",
                                kyon_urdf_folder + "/urdf/kyon.urdf.xacro",
                                "sensors:=false",
                                "upper_body:=false",
                                "wheels:=true",
                                "payload:=false"])

srdf = subprocess.check_output(["xacro",
                                kyon_srdf_folder + "/srdf/kyon.srdf.xacro",
                                "sensors:=false",
                                "upper_body:=false",
                                "wheels:=true",
                                "payload:=false"])
urdf = urdf.decode('utf-8')
srdf = srdf.decode('utf-8')

file_dir = os.getcwd()

'''
Initialize Horizon problem
'''
ns = 30
T = 3
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

urdf = urdf.replace('continuous', 'revolute')
kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

q_init = {'hip_roll_1': 0.0,
          'hip_pitch_1': 0.7,
          'knee_pitch_1': -1.4,
          'hip_roll_2': 0.0,
          'hip_pitch_2': 0.7,
          'knee_pitch_2': -1.4,
          'hip_roll_3': 0.0,
          'hip_pitch_3': -0.7,
          'knee_pitch_3': 1.4,
          'hip_roll_4': 0.0,
          'hip_pitch_4': -0.7,
          'knee_pitch_4': 1.4,
          'wheel_joint_1': 0.0,
          'wheel_joint_2': 0.0,
          'wheel_joint_3': 0.0,
          'wheel_joint_4': 0.0}

# q_init = {'hip_roll_1': 0.0,
#           'hip_pitch_1': -0.3,
#           'knee_pitch_1': -0.92,
#           'hip_roll_2': 0.0,
#           'hip_pitch_2': -0.3,
#           'knee_pitch_2': -0.92,
#           'hip_roll_3': 0.0,
#           'hip_pitch_3': -0.6,
#           'knee_pitch_3': 1.26,
#           'hip_roll_4': 0.0,
#           'hip_pitch_4': -0.6,
#           'knee_pitch_4': 1.26,
#           'wheel_joint_1': 0.0,
#           'wheel_joint_2': 0.0,
#           'wheel_joint_3': 0.0,
#           'wheel_joint_4': 0.0}

# 'shoulder_yaw_1': 0.0,
# 'shoulder_pitch_1': 0.9,
# 'elbow_pitch_1': 1.68,
# 'wrist_pitch_1': 0.,
# 'wrist_yaw_1': 0.,
# 'shoulder_yaw_2': 0.0,
# 'shoulder_pitch_2': 0.9,
# 'elbow_pitch_2': 1.68,
# 'wrist_pitch_2': 0.,
# 'wrist_yaw_2': 0.}

base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
# base_init = np.array([0.0, 0.0, 0.0, 0.014192, 0.0297842, -0.0230466, 0.9991898])

FK = kin_dyn.fk('ball_1')
init = base_init.tolist() + list(q_init.values())
init_pos_foot = FK(q=init)['ee_pos']
base_init[2] = -init_pos_foot[2]

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init,
                                 )

rospy.set_param('mpc/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=mpc/robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(file_dir + '/../config/kyon_to_config.yaml')

com_height = ti.getTask('com_height')
com_height.setRef(np.atleast_2d(base_init).T)

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_phases = dict()
for c in model.cmap.keys():
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

zmp_timeline = pm.addTimeline('zmp_timeline')

def zmp(model):
    # formulation in forces
    num = cs.SX([0, 0])
    den = cs.SX([0])

    q = cs.SX.sym('q', model.nq)
    v = cs.SX.sym('v', model.nv)
    a = cs.SX.sym('a', model.nv)

    pos_contact = dict()
    force_val = dict()

    # tau_f_res = cs.SX([0, 0, 0])
    # f_res = cs.SX([0, 0, 0])
    #
    # for c in model.fmap.keys():
    #     pos_contact[c] = cs.SX.sym('pos_contact', 3)
    #     force_val[c] = cs.SX.sym('force_val', 3)
    #
    # for c in model.fmap.keys():
    #     tau_f_res += cs.cross(pos_contact[c], force_val[c])
    #     f_res += force_val[c]
    #
    # n = cs.SX([0, 0, 1])
    #
    # zmp = cs.cross(n, tau_f_res) / (cs.dot(f_res, n))
    #
    # input_list = []
    # for elem in pos_contact.values():
    #     input_list.append(elem)
    #
    # for elem in force_val.values():
    #     input_list.append(elem)

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

input_zmp = []
# for c_name in model.fmap.keys():
#     input_zmp.append(kin_dyn.fk(c_name)(q=model.q)['ee_pos'])

input_zmp.append(model.q)
input_zmp.append(model.v)
input_zmp.append(model.a)

for f_var in model.fmap.values():
    input_zmp.append(f_var)

c_mean = cs.SX([0, 0, 0])
for c_name, f_var in model.fmap.items():
    fk_c_pos = kin_dyn.fk(c_name)(q=model.q)['ee_pos']
    c_mean += fk_c_pos

c_mean /= len(model.cmap.keys())

# zmp_weight = prb.createParameter('zmp_weight', 1)
zmp_nominal_weight = 10.
# zmp_weight.assign(zmp_nominal_weight)
zmp_fun = zmp(model)(*input_zmp)
zmp = prb.createIntermediateResidual('zmp',  zmp_nominal_weight * (zmp_fun[0:2] - c_mean[0:2]), nodes=[])
zmp_empty = prb.createIntermediateResidual('zmp_empty', 0. * (zmp_fun[0:2] - c_mean[0:2]), nodes=[])

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

# register zmp phase
zmp_phase = pyphase.Phase(stance_duration, 'zmp_phase')
zmp_phase.addCost(zmp)
zmp_empty_phase = pyphase.Phase(flight_duration, 'zmp_empty_phase')
zmp_empty_phase.addCost(zmp_empty)
zmp_timeline.registerPhase(zmp_phase)
zmp_timeline.registerPhase(zmp_empty_phase)

# pos_lf = model.kd.fk('l_sole')(q=model.q)['ee_pos']
# pos_rf = model.kd.fk('r_sole')(q=model.q)['ee_pos']
# base_ori = horizon.utils.utils.toRot(model.kd.fk('base_link')(q=model.q0)['ee_rot'])
# rel_dist = base_ori.T @ (pos_lf - pos_rf)

# prb.createResidual('relative_distance_lower_x', horizon.utils.utils.barrier(rel_dist[0] + 0.3))
# prb.createResidual('relative_distance_upper_x', horizon.utils.utils.barrier1(rel_dist[0] - 0.4))
# prb.createResidual('relative_distance_lower_y', 10 * horizon.utils.utils.barrier(rel_dist[1] - 0.2))
# prb.createResidual('relative_distance_upper_y', horizon.utils.utils.barrier1(rel_dist[1] + 1.))

# for f_name, f in model.fmap.items():
#     f_prev = f.getVarOffset(-1)
#     prb.createIntermediateResidual(f'{f_name}_smooth_forces', 1e-2 * (f_prev - f), nodes=range(1, ns-1))

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
    flight = c_phases[c].getRegisteredPhase(f'flight_{c}')
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)
    c_phases[c].addPhase(stance)
    # zmp_timeline.addPhase(zmp_phase)

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
# ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
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
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)
                                           # future_trajectory_markers={'base_link': 'world', 'ball_1': 'world'})

global joy_msg

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()

from joy_commands import GaitManager, JoyCommands
contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

jc = JoyCommands(gm)

from geometry_msgs.msg import PointStamped
zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)

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
    pm._shift_phases()
    time_elapsed_shifting = time.time() - tic
    time_elapsed_shifting_list.append(time_elapsed_shifting)

    jc.run(solution)

    iteration = iteration + 1

    ti.rti()
    solution = ti.solution
    dt_res = 0.01
    ti.resample(dt_res=dt_res, nodes=[0, 1], resample_tau=False)

    tau = list()

    for i in range(solution['q_res'].shape[1] - 1):
        tau.append(ti.model.computeTorqueValues(solution['q_res'][:, i], solution['v_res'][:, i], solution['a_res'][:, i],
                                                {name: solution['f_' + name][:, i] for name in model.fmap}))

    jt = JointTrajectory()
    for i in range(solution['q_res'].shape[1]):
        jtp = JointTrajectoryPoint()
        jtp.positions = solution['q_res'][:, i].tolist()
        jtp.velocities = solution['v_res'][:, i].tolist()
        if i < len(tau):
            jtp.accelerations = solution['a_res'][:, i].tolist()
            jtp.effort = tau[i].elements()
        else:
            jtp.accelerations = solution['a_res'][:, -1].tolist()
            jtp.effort = tau[-1].elements()

        jt.points.append(jtp)


    jt.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]
    jt.header.stamp = rospy.Time.now()

    solution_publisher.publish(jt)

    try:
        print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
        print(prb.getCosts('zmp').getNodes())
    except:
        pass

    # replay stuff
    repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
    repl.publish_joints(solution['q'][:, 0])
    repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
    # repl.publish_future_trajectory_marker('base_link', solution['q'][0:3, :])
    # repl.publish_future_trajectory_marker('ball_1', solution['q'][8:11, :])
    rate.sleep()

print(f'average time elapsed shifting: {sum(time_elapsed_shifting_list) / len(time_elapsed_shifting_list)}')
