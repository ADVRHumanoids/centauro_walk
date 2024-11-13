#!/usr/bin/python3

from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from horizon.ros import replay_trajectory
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
import horizon.utils.analyzer as analyzer
from geometry_msgs.msg import PointStamped

from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import tf

import horizon.utils as utils

import convex_plane_decomposition_ros.pysegmented_plane_projection as projector


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
ns = 280
T = 12.
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



base_pose = [0., 0., 0., 0., 0., 0., 1]
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

pub_dict = dict()
pub_dict['contact_1_query'] = rospy.Publisher('~contact_1_query', PointStamped, queue_size=1)
pub_dict['contact_1_proj'] = rospy.Publisher('~contact_1_proj', PointStamped, queue_size=1)
pub_dict['contact_2_query'] = rospy.Publisher('~contact_2_query', PointStamped, queue_size=1)
pub_dict['contact_2_proj'] = rospy.Publisher('~contact_2_proj', PointStamped, queue_size=1)
pub_dict['contact_3_query'] = rospy.Publisher('~contact_3_query', PointStamped, queue_size=1)
pub_dict['contact_3_proj'] = rospy.Publisher('~contact_3_proj', PointStamped, queue_size=1)
pub_dict['contact_4_query'] = rospy.Publisher('~contact_4_query', PointStamped, queue_size=1)
pub_dict['contact_4_proj'] = rospy.Publisher('~contact_4_proj', PointStamped, queue_size=1)
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


FK = kin_dyn.fk('contact_1')

init_pos_foot = FK(q=kin_dyn.mapToQ(q_init))['ee_pos'].elements()
base_pose[2] = -init_pos_foot[2]
base_pose[0] = 1.3

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
c_timelines = dict()
for c in model.cmap.keys():
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')

FK_contacts = dict()
dFK_contacts = dict()
for c in model.getContactMap():
    FK_contacts[c] = model.kd.fk(c)

    dFK_contacts[c] = model.kd.frameVelocity(c, model.kd_frame)


short_stance_duration = 5
stance_duration = 15
flight_duration = 15
c_i = 0

for c in model.getContactMap():
    c_i += 1  # because contact task start from contact_1
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_crawl_{c}')
    stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
    if ti.getTask(f'contact_{c_i}') is not None:
        stance_phase.addItem(ti.getTask(f'contact_{c_i}'))
        stance_phase_short.addItem(ti.getTask(f'contact_{c_i}'))
    else:
        raise Exception('task not found')


    # flight phase normal
    flight_phase = c_timelines[c].createPhase(flight_duration, f'crawl_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]

    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.1, [None, 0, None]))
    if ti.getTask(f'z_contact_{c_i}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_contact_{c_i}'), ref_trj)
    else:
        raise Exception('task not found')

    ee_vel = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

    c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
    cost_ori = prb.createResidual(f'{c}_ori', 5. * (c_ori.T - np.array([0, 0, 1])), nodes=[])
    flight_phase.addCost(cost_ori, nodes=[flight_duration-1])

    ref_trj_xy = np.zeros(shape=[7, 1])
    flight_phase.addItemReference(ti.getTask(f'xy_contact_{c_i}'), ref_trj_xy, nodes=[flight_duration - 1])


contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

gm.stand() # 5 nodes
gm.stand() # 5 nodes
gm.stand() # 5 nodes
gm.crawl()
gm.crawl()
gm.crawl()
gm.stand() # 5 nodes
gm.stand() # 5 nodes
gm.stand() # 5 nodes
gm.stand() # 5 nodes
gm.stand() # 5 nodes

# for c in model.cmap.keys():
#     stance = c_timelines[c].getRegisteredPhase(f'stance_crawl_{c}')
#     while c_timelines[c].getEmptyNodes() > 0:
#         c_timelines[c].addPhase(stance)

# for c in model.cmap.keys():
#     print(f'timeline {c}:')
#     for phase in c_timelines[c].getActivePhases():
#         print(phase.getName(), end=" ")
#     print()

q_final = ti.model.q0.copy()
q_final[0] += 0.9
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

pm.update()
ti.bootstrap()

solution = ti.solution



# contact_list_repl = list(model.cmap.keys())
# repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), solution['q'],
#                                            {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()},
#                                            model.kd_frame, model.kd,
#                                            # trajectory_markers=contact_list_repl,
#                                            fixed_joint_map=fixed_joint_map)
#
# repl.replay()

# second solve to project contact onto the extracted polygons


for c in model.getContactMap():
    if ti.getTask(f'xy_{c}') is None:
        raise Exception(f'xy_{c} task not defined!')
    ti.getTask(f'xy_{c}').setWeight(100.)

projected_initial_pose_list = dict()
projected_final_pose_list = dict()
original_landing_pose_list = dict()

for n_step in range(3):
    projected_initial_pose_list[n_step] = dict()
    projected_final_pose_list[n_step] = dict()
    original_landing_pose_list[n_step] = dict()

for c, timeline in c_timelines.items():

    n_step = 0

    for phase in timeline.getActivePhases():
        if phase.getName() == f'crawl_{c}':

            final_node = phase.getPosition() + phase.getNNodes()
            if final_node < ns:
                q_init = solution['q'][:, phase.getPosition()]
                q_fin = solution['q'][:, final_node]

                initial_pose = FK_contacts[c](q=q_init)['ee_pos'].elements()
                projected_initial_pose = projector.project(initial_pose)
                landing_pose = FK_contacts[c](q=q_fin)['ee_pos'].elements()
                projected_final_pose = projector.project(landing_pose)

                # if 0.02 < np.linalg.norm(qp - pp) < 0.1:
                ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration,
                                                                  projected_initial_pose[2],
                                                                  projected_final_pose[2],
                                                                  0.1,
                                                                  [None, 0, None]))

                phase.setItemReference(f'z_{c}', ref_trj)

                # phase.setItemWeight(f'xy_{c}', [50.])
                ref_trj_xy[0:2, 0] = projected_final_pose[0:2]
                phase.setItemReference(f'xy_{c}', ref_trj_xy)
                pm.update()

                projected_initial_pose_list[n_step][c] = projected_initial_pose
                projected_final_pose_list[n_step][c] = projected_final_pose
                original_landing_pose_list[n_step][c] = landing_pose



                n_step += 1


# rate = rospy.Rate(100)
# while not rospy.is_shutdown():
#     for n_step in range(3):
#         for i_loop in range(100):
#             for c, timeline in c_timelines.items():
#
#                 query_point = PointStamped()
#                 query_point.header.frame_id = 'world'
#                 query_point.header.stamp = rospy.Time.now()
#
#                 query_point.point.x = original_landing_pose_list[n_step][c][0]
#                 query_point.point.y = original_landing_pose_list[n_step][c][1]
#                 query_point.point.z = original_landing_pose_list[n_step][c][2]
#
#                 projected_point = PointStamped()
#                 projected_point.header = query_point.header
#                 projected_point.point.x = projected_final_pose_list[n_step][c][0]
#                 projected_point.point.y = projected_final_pose_list[n_step][c][1]
#                 projected_point.point.z = projected_final_pose_list[n_step][c][2]
#
#                 pub_dict[f'{c}_query'].publish(query_point)
#                 pub_dict[f'{c}_proj'].publish(projected_point)
#                 rate.sleep()

                # qp = np.array([query_point.point.x, query_point.point.y])  # , query_point.point.z])
                # pp = np.array([projected_point.point.x, projected_point.point.y])  # , projected_point.point.z])




prb.getState().setInitialGuess(solution['x_opt'])

# anal = analyzer.ProblemAnalyzer(ti.prb)
# anal.printParameters()
# exit()

ti.bootstrap()

solution = ti.solution


# ============================================= send to gazebo ============================================
print('sending it to gazebo...')
from kyon_controller.msg import WBTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)

sol_msg = WBTrajectory()
sol_msg.header.frame_id = 'world'
sol_msg.header.stamp = rospy.Time.now()

sol_msg.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

rate = rospy.Rate(1/dt)
for i_sample in range(ns):
    sol_msg.q = solution['q'][:, i_sample].tolist()
    sol_msg.v = solution['v'][:, i_sample].tolist()
    sol_msg.a = solution['a'][:, i_sample].tolist()

    for frame in model.getForceMap():
        sol_msg.force_names.append(frame)
        sol_msg.f.append(
            Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))

    solution_publisher.publish(sol_msg)
    rate.sleep()

# ========================================================-------============================================

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), solution['q'],
                                           {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl,
                                           fixed_joint_map=fixed_joint_map)

repl.replay()

projector.shutdown()