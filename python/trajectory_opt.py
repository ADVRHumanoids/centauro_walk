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

from joy_commands import GaitManager
from geometry_msgs.msg import PointStamped
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

# solution_publisher = rospy.Publisher('/mpc_solution', JointTrajectory, queue_size=10)

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
ns = 50
T = 5.
dt = T / ns

prb = Problem(ns, receding=True) # casadi_type=cs.SX
prb.setDt(dt)

urdf = urdf.replace('continuous', 'revolute')
kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

q_init = {'hip_roll_1': 0.0,
          'hip_pitch_1': -0.3,
          'knee_pitch_1': -0.92,
          'hip_roll_2': 0.0,
          'hip_pitch_2': -0.3,
          'knee_pitch_2': -0.92,
          'hip_roll_3': 0.0,
          'hip_pitch_3': 0.6,
          'knee_pitch_3': -1.26,
          'hip_roll_4': 0.0,
          'hip_pitch_4': 0.6,
          'knee_pitch_4': -1.26,
          'wheel_joint_1': 0.0,
          'wheel_joint_2': 0.0,
          'wheel_joint_3': 0.0,
          'wheel_joint_4': 0.0}

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

base_init = np.array([0.0, 0.0, 0.0, 0.014192, 0.0297842, -0.0230466, 0.9991898])

FK = kin_dyn.fk('ball_1')
init = base_init.tolist() + list(q_init.values())
init_pos_foot = FK(q=init)['ee_pos']
base_init[2] = -init_pos_foot[2]

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init,
                                 )

rospy.set_param('robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(file_dir + '/../config/kyon_to_config.yaml')

# com_height = ti.getTask('com_height')
# com_height.setRef(np.atleast_2d(base_init).T)

xy_goal = ti.getTask('final_base_xy')
xy_goal.setRef(np.array([[6., 5., 0., 0., 0., 0., 0.]]).T)

base_orientation = ti.getTask('base_orientation')
base_orientation.setRef(np.array([[0., 0., 0., 0., 0., 0.707, 0.707]]).T)

# base_orientation = ti.getTask('base_orientation')
# base_orientation.setRef(np.array([[0., 0., 0., 0., 0., 0.38, 0.92]]).T)


pos_lf = model.kd.fk('wheel_1')(q=model.q)['ee_pos']
pos_rf = model.kd.fk('wheel_2')(q=model.q)['ee_pos']
pos_lh = model.kd.fk('wheel_3')(q=model.q)['ee_pos']
pos_rh = model.kd.fk('wheel_4')(q=model.q)['ee_pos']

dist_weight = 1.
rel_dist = 0.6
prb.createConstraint('relative_distance_l', utils.utils.barrier(pos_lf[0] - pos_lh[0] - rel_dist))
prb.createConstraint('relative_distance_r', utils.utils.barrier(pos_rf[0] - pos_rh[0] - rel_dist))


# for c_name in ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']:
#     # rot_vec = self._rotate_vector(vec, solution['q'][[6, 3, 4, 5], 0])
#
# exit()
def zmp(model):

    # formulation in forces
    tau_f_res = cs.SX([0, 0, 0])
    f_res = cs.SX([0, 0, 0])
    pos_contact = dict()
    force_val = dict()
    for c in model.fmap.keys():
        pos_contact[c] = cs.SX.sym('pos_contact', 3)
        force_val[c] = cs.SX.sym('force_val', 3)

    for c in model.fmap.keys():
        tau_f_res += cs.cross(pos_contact[c], force_val[c])
        f_res += force_val[c]

    n = cs.SX([0, 0, 1])

    zmp = cs.cross(n, tau_f_res) / (cs.dot(f_res, n))

    input_list = []
    for elem in pos_contact.values():
        input_list.append(elem)

    for elem in force_val.values():
        input_list.append(elem)

    f = cs.Function('zmp', input_list, [zmp])

    return f

input_zmp = []
for c_name in model.fmap.keys():
    input_zmp.append(kin_dyn.fk(c_name)(q=model.q)['ee_pos'])

for f_var in model.fmap.values():
    input_zmp.append(f_var)

zmp_fun = zmp(model)(*input_zmp)

c_mean = cs.SX([0, 0, 0])
for c_name, f_var in model.fmap.items():
    fk_c_pos = kin_dyn.fk(c_name)(q=model.q)['ee_pos']
    c_mean += fk_c_pos

c_mean /= len(model.cmap.keys())

zmp_weight = 20.
zmp = prb.createIntermediateConstraint('zmp',  zmp_weight * (zmp_fun[0:2] - c_mean[0:2]))


tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_phases = dict()
for c in model.cmap.keys():
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

for c in model.cmap.keys():
    # stance phase normal
    stance_duration = 5
    stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    if ti.getTask(f'{c}_contact') is not None:
        stance_phase.addItem(ti.getTask(f'{c}_contact'))
    else:
        raise Exception('task not found')

    c_phases[c].registerPhase(stance_phase)

    # flight phase normal
    flight_duration = 5
    flight_phase = pyphase.Phase(flight_duration, f'flight_flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.05, [None, 0, None]))
    if ti.getTask(f'z_{c}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
    else:
        raise Exception('task not found')
    # flight_phase.addConstraint(prb.getConstraints(f'{c}_vert'), nodes=[0 ,flight_duration-1])  # nodes=[0, 1, 2]
    c_phases[c].registerPhase(flight_phase)


for c in model.cmap.keys():
    stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
    flight = c_phases[c].getRegisteredPhase(f'flight_{c}')
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)
    c_phases[c].addPhase(stance)

for c in model.cmap.keys():
    if c == 'ball_1':
        flight = c_phases[c].getRegisteredPhase(f'flight_{c}')
        c_phases[c].addPhase(flight, 2)



ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

# joint posture
black_list_indices = list()
black_list = ['wheel_joint_1', 'wheel_joint_2', 'wheel_joint_3', 'wheel_joint_4']
postural_joints = np.array(list(range(7, model.nq)))
for joint in black_list:
    black_list_indices.append(model.joint_names.index(joint))
postural_joints = np.delete(postural_joints, black_list_indices)

# ti.model.q[postural_joints].setBounds(ti.model.q0[postural_joints], ti.model.q0[postural_joints], nodes=ns)
ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=ns)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

# finalize taskInterface and solve bootstrap problem
ti.finalize(rti=False)

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution
ti.resample(0.001)
ti.replay_trajectory()


# zmp publishing
# zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)
# zmp_fun_val = prb.getConstraints('zmp_val')
# all_vars = list()
# for var in zmp_fun_val.getVariables():
#     var_name = var.getName()
#
#     node_index = range(ns)
#     all_vars.append(solution[var_name][:, node_index])
#
# zmp_sol = zmp_fun_val.getFunction()(*all_vars)
# zmp_point = PointStamped()
# zmp_point.header.stamp = rospy.Time.now()
# zmp_point.header.frame_id = "world"
# zmp_point.point.x = zmp_sol[0]
# zmp_point.point.y = zmp_sol[1]
# zmp_point.point.z = 0.
#
# zmp_pub.publish(zmp_point)

