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


rospy.init_node('centauro_wheels')
roscpp.init('centauro_wheels', [])

solution_publisher = rospy.Publisher('/mpc_solution', JointTrajectory, queue_size=10)
rospy.sleep(1.)

'''
Load urdf and srdf
'''

centauro_urdf_folder = rospkg.RosPack().get_path('centauro_urdf')
centauro_srdf_folder = rospkg.RosPack().get_path('centauro_srdf')


urdf_path = centauro_urdf_folder + '/urdf/centauro.urdf'
urdf = open(urdf_path, 'r').read()
srdf_path = centauro_srdf_folder + '/urdf/centauro.urdf'
srdf = open(urdf_path, 'r').read()

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

q_init = {"ankle_pitch_1": -0.30166,
          "ankle_pitch_2": 0.301666,
          "ankle_pitch_3": 0.301667,
          "ankle_pitch_4": -0.30166,
          "ankle_yaw_1": 0.746874,
          "ankle_yaw_2": -0.74687,
          "ankle_yaw_3": -0.74687,
          "ankle_yaw_4": 0.746874,
          "d435_head_joint": 0,
          "hip_pitch_1": -1.25409,
          "hip_pitch_2": 1.25409,
          "hip_pitch_3": 1.25409,
          "hip_pitch_4": -1.25409,
          "hip_yaw_1": -0.746874,
          "hip_yaw_2": 0.746874,
          "hip_yaw_3": 0.746874,
          "hip_yaw_4": -0.746874,
          "j_arm1_1": 0.520149,
          "j_arm1_2": 0.320865,
          "j_arm1_3": 0.274669,
          "j_arm1_4": -2.23604,
          "j_arm1_5": 0.0500815,
          "j_arm1_6": -0.781461,
          "j_arm2_1": 0.520149,
          "j_arm2_2": -0.320865,
          "j_arm2_3": -0.274669,
          "j_arm2_4": -2.23604,
          "j_arm2_5": -0.0500815,
          "j_arm2_6": -0.781461,
          "knee_pitch_1": -1.55576,
          "knee_pitch_2": 1.55576,
          "knee_pitch_3": 1.55576,
          "knee_pitch_4": -1.55576,
          "torso_yaw": 3.56617e-13,
          "velodyne_joint": 0,
          "j_wheel_1": 0,
          "j_wheel_2": 0,
          "j_wheel_3": 0,
          "j_wheel_4": 0}

base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

FK = kin_dyn.fk('contact_1')
init = base_init.tolist() + list(q_init.values())
init_pos_foot = FK(q=init)['ee_pos']
base_init[2] = -init_pos_foot[2]

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init,
                                 )


print(model.joint_names)
# exit()

rospy.set_param('robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(file_dir + '/../config/centauro_config.yaml')

# final_base_xy = ti.getTask('final_base_xy')
# base_init[0] += 0.
# base_init[1] += 2.
# final_base_xy.setRef(np.atleast_2d(base_init).T)

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
# zmp = prb.createIntermediateResidual('zmp',  zmp_nominal_weight * (zmp_fun[0:2] - c_mean[0:2]))
# zmp_empty = prb.createIntermediateResidual('zmp_empty', 0. * (zmp_fun[0:2] - c_mean[0:2]), nodes=[])

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


# register zmp phase
# zmp_phase = pyphase.Phase(stance_duration, 'zmp_phase')
# zmp_phase.addCost(zmp)
# zmp_empty_phase = pyphase.Phase(flight_duration, 'zmp_empty_phase')
# zmp_empty_phase.addCost(zmp_empty)
# zmp_timeline.registerPhase(zmp_phase)
# zmp_timeline.registerPhase(zmp_empty_phase)

for c in model.cmap.keys():
    stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
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

q_init_guess = dict(q_init)
for value in q_init_guess.values():
    value = 0.

ti.model.q.setInitialGuess(model.q0)
ti.model.v.setInitialGuess(model.v0)

black_list_indices = list()
black_list = ['j_wheel_1', 'j_wheel_2', 'j_wheel_3', 'j_wheel_4']
postural_joints = np.array(list(range(6, model.nv)))
for joint in black_list:
    black_list_indices.append(model.joint_names.index(joint))
postural_joints = np.delete(postural_joints, black_list_indices)
prb.createResidual('v_regularizaiton', 1. * model.v[postural_joints])


f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

# finalize taskInterface and solve bootstrap problem
ti.finalize()

ti.bootstrap()
# ti.resample(0.001)
# ti.replay_trajectory()

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

    # xig[6 + 5] = q_init['ankle_yaw_1'] + jc.direction
    # xig[6 + 11] = q_init['ankle_yaw_2'] + jc.direction
    # xig[6 + 17] = q_init['ankle_yaw_3'] + jc.direction
    # xig[6 + 23] = q_init['ankle_yaw_4'] + jc.direction

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

    # replay stuff
    repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
    repl.publish_joints(solution['q'][:, 0])
    repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
    # repl.publish_future_trajectory_marker('base_link', solution['q'][0:3, :])
    # repl.publish_future_trajectory_marker('ball_1', solution['q'][8:11, :])
    rate.sleep()

print(f'average time elapsed shifting: {sum(time_elapsed_shifting_list) / len(time_elapsed_shifting_list)}')
