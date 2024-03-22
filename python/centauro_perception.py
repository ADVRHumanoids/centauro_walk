#!/usr/bin/python3

from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from horizon.ros import replay_trajectory
from horizon.utils.resampler_trajectory import Resampler
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import cartesian_interface.roscpp_utils as roscpp
import cartesian_interface.pyci as pyci
import cartesian_interface.affine3
import horizon.utils.analyzer as analyzer

from base_estimation import pybase_estimation
from base_estimation.msg import ContactWrenches
from geometry_msgs.msg import Wrench

from scipy.spatial.transform import Rotation

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from geometry_msgs.msg import PointStamped, TwistStamped, Vector3
from kyon_controller.msg import WBTrajectory

from centauro_joy_commands import GaitManager

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess

import horizon.utils as utils

import convex_plane_decomposition.pysegmented_plane_projection as projector


rospy.init_node('centauro_perception')

projector.init('centauro_perception', [])

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
ns = 110
T = 6.
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

q_init = {'hip_yaw_1': -0.746,
          'hip_pitch_1': -1.254,
          'knee_pitch_1': -1.555,
          'ankle_pitch_1': -0.3,

          'hip_yaw_2': 0.746,
          'hip_pitch_2': 1.254,
          'knee_pitch_2': 1.555,
          'ankle_pitch_2': 0.3,

          'hip_yaw_3': 0.746,
          'hip_pitch_3': 1.254,
          'knee_pitch_3': 1.555,
          'ankle_pitch_3': 0.3,

          'hip_yaw_4': -0.746,
          'hip_pitch_4': -1.254,
          'knee_pitch_4': -1.555,
          'ankle_pitch_4': -0.3}

base_pose = [0.07, 0., 0.8, 0., 0., 0., 1]
base_twist = np.zeros(6)

wheels = [f'j_wheel_{i + 1}' for i in range(4)]
wheels_map = dict(zip(wheels, 4 * [0.]))

ankle_yaws = [f'ankle_yaw_{i + 1}' for i in range(4)]
ankle_yaws_map = dict(zip(ankle_yaws, [np.pi/4, -np.pi/4, -np.pi/4, np.pi/4]))

arm_joints = [f'j_arm1_{i + 1}' for i in range(6)] + [f'j_arm2_{i + 1}' for i in range(6)]
arm_joints_map = dict(zip(arm_joints, [0.75, 0.1, 0.2, -2.2, 0., -1.3, 0.75, 0.1, -0.2, -2.2, 0.0, -1.3]))

torso_map = {'torso_yaw': 0.}

head_map = {'d435_head_joint': 0.0, 'velodyne_joint': 0.0}

fixed_joint_map = dict()
fixed_joint_map.update(wheels_map)
fixed_joint_map.update(ankle_yaws_map)
fixed_joint_map.update(arm_joints_map)
fixed_joint_map.update(torso_map)
fixed_joint_map.update(head_map)

# query_pub = rospy.Publisher('~query_point', PointStamped, queue_size=1)
# projected_pub = rospy.Publisher('~projected_point', PointStamped, queue_size=1)
#
# query_point = PointStamped()
# query_point.header.frame_id = 'odom_offset'
# query_point.header.stamp = rospy.Time.now()
# query_point.point.x = 0.
# query_point.point.y = 0.
# query_point.point.z = 0.
#
# prj = projector.project(np.array([query_point.point.x, query_point.point.y, query_point.point.z]))
#
# projected_point = PointStamped()
# projected_point.header = query_point.header
# projected_point.point.x = prj[0]
# projected_point.point.y = prj[1]
# projected_point.point.z = prj[2]
#
# r = rospy.Rate(100)
# while not rospy.is_shutdown():
#     query_pub.publish(query_point)
#     projected_pub.publish(projected_point)
#     r.sleep()

# replace continuous joints with revolute
urdf = urdf.replace('continuous', 'revolute')

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joint_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 fixed_joint_map=fixed_joint_map
                                 )

bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/centauro_perception.yaml')

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)

# phase manager handling
c_phases = dict()
for c in model.cmap.keys():
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

ground = dict()

short_stance_duration = 5
stance_duration = 15
flight_duration = 15
c_i = 0

for c in model.getContactMap():
    c_i += 1  # because contact task start from contact_1
    # stance phase normal
    stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    stance_phase_short = pyphase.Phase(short_stance_duration, f'stance_{c}_short')
    if ti.getTask(f'contact_{c_i}') is not None:
        stance_phase.addItem(ti.getTask(f'contact_{c_i}'))
        stance_phase_short.addItem(ti.getTask(f'contact_{c_i}'))
    else:
        raise Exception('task not found')

    c_phases[c].registerPhase(stance_phase)
    c_phases[c].registerPhase(stance_phase_short)

    # flight phase normal
    flight_phase = pyphase.Phase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ground[c] = prb.createParameter(f'ground_{c}', 1)
    ground[c].assign(init_z_foot)

    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.casadi_trajectory(flight_duration, ground[c], ground[c], 0.1, [1000, 0, 1000]))
    if ti.getTask(f'z_contact_{c_i}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_contact_{c_i}'), ref_trj)
    else:
        raise Exception('task not found')

    ee_vel = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

    c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
    cost_ori = prb.createResidual(f'{c}_ori', 5. * (c_ori.T - np.array([0, 0, 1])))
    flight_phase.addCost(cost_ori)

    c_phases[c].registerPhase(flight_phase)

contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

gm.stand()
gm.crawl()
gm.stand()

q_final = ti.model.q0.copy()
q_final[0] += 0.3
ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.q[0:7].setBounds(q_final[0:7], q_final[0:7], nodes=ns)
ti.model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=0)
ti.model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=range(ns-7, ns))

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

solution = ti.solution

# second solve to project contact onto the extracted polygons

for c, timeline in c_phases.items() :
    for phase in timeline.getActivePhases():
        if phase.getName() == f'flight_{c}':
            final_node = phase.getPosition() + phase.getNNodes()
            initial_pose = model.kd.fk(c)(q=solution['q'][:, phase.getPosition()])['ee_pos'].elements()
            landing_pose = model.kd.fk(c)(q=solution['q'][:, final_node])['ee_pos'].elements()
            projected_pose = projector.project(landing_pose)
            ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, initial_pose[2], projected_pose[2], 0.1, [None, 0, None]))
            phase.addItemReference(ti.getTask(f'z_contact_{c_i}'), ref_trj)

ti.finalize()

ti.bootstrap()

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), solution['q'],
                                           {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl,
                                           fixed_joint_map=fixed_joint_map)

repl.replay(is_floating_base=True)

projector.shutdown()