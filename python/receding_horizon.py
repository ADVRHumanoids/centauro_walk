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


rospy.init_node('cogimon_walk_srbd')
roscpp.init('cogimon_walk_srbd', [])

# logger = matlogger.MatLogger2('/tmp/srbd_bootstrap')

solution_publisher = rospy.Publisher('/mpc_solution', JointTrajectory, queue_size=10)
rospy.sleep(1.)

'''
Load urdf and srdf
'''

cetc_sat_urdf_folder = rospkg.RosPack().get_path('cetc_sat_urdf')
cetc_sat_srdf_folder = rospkg.RosPack().get_path('cetc_sat_srdf')

urdf = subprocess.check_output(["xacro",
                                cetc_sat_urdf_folder + "/urdf/cetc_sat.urdf.xacro",
                                "sensors:=false",
                                "upper_body:=false",
                                "bilevel_codes:=false",
                                "payload:=false"])

srdf = subprocess.check_output(["xacro",
                                cetc_sat_srdf_folder + "/srdf/cetc_sat.srdf.xacro",
                                "sensors:=false",
                                "upper_body:=false",
                                "bilevel_codes:=false",
                                "payload:=false"])
urdf = urdf.decode('utf-8')
srdf = srdf.decode('utf-8')

file_dir = os.getcwd()

'''
Initialize Horizon problem
'''
ns = 15
T = 1.5
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

contact_dict = {
    'ball_1': {
        'type': 'point'
    },

    'ball_2': {
        'type': 'point'
    },

    'ball_3': {
        'type': 'point'
    },

    'ball_4': {
        'type': 'point'
    },
}

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

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
          'knee_pitch_4': -1.4}
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
ti.setTaskFromYaml(file_dir + '/../config/kyon_horizon_config.yaml')
ti.setTaskFromYaml(file_dir + '/../config/config_demo.yaml')

com_height = ti.getTask('com_height')
com_x = ti.getTask('final_base_x')
com_y = ti.getTask('final_base_y')

com_height.setRef(np.atleast_2d(base_init).T)

contact_ori = dict()
for c_name in contact_dict.keys():
    c_rot = model.kd.fk(c_name)(q=model.q0)['ee_rot'].toarray()
    c_quat = utils.utils.matrix_to_quaternion(c_rot)
    contact_ori[c_name] = ti.getTask(f'{c_name}_orientation')
    contact_ori[c_name].setRef(np.array([[0., 0., 0., c_quat[0], c_quat[1], c_quat[2], c_quat[3]]]).T)

# contact_xy = dict()
# for c_name in contact_dict.keys():
#     c_pos_xy = model.kd.fk(c_name)(q=model.q0)['ee_pos'].toarray()
#     print(c_pos_xy[0])
#     contact_xy[c_name] = ti.getTask(f'xy_{c_name}')
#     contact_xy[c_name].setRef(np.array([[c_pos_xy[0][0], c_pos_xy[1][0], c_pos_xy[2][0], 0., 0., 0., 0.]]).T)

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_phases = dict()
for c in contact_dict:
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

for c in contact_dict:
    # stance phase normal
    stance_duration = 3
    stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    stance_phase.addItem(ti.getTask(f'{c}_contact'))
    c_phases[c].registerPhase(stance_phase)

    # flight phase normal
    flight_duration = 3
    flight_phase = pyphase.Phase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.1, [None, 0, None]))
    flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
    c_phases[c].registerPhase(flight_phase)

    # stance phase short
    # stance_duration = 1
    # stance_phase = pyphase.Phase(stance_duration, f'stance_{c}_short')
    # stance_phase.addItem(ti.getTask(f'{c}_contact'))
    # c_phases[c].registerPhase(stance_phase)

    # flight phase short
    # flight_duration = 1
    # flight_phase = pyphase.Phase(flight_duration, f'flight_{c}_short')
    # init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    # ref_trj = np.zeros(shape=[7, flight_duration])
    # ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.03, [None, 0, None]))
    # flight_phase.addItemReference(ti.getTask(f'z_{c}'), ref_trj)
    # c_phases[c].registerPhase(flight_phase)

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

for c in contact_dict:
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

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
# ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

f0 = [0, 0, kin_dyn.mass() / 8 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

# finalize taskInterface and solve bootstrap problem
ti.finalize()

anal = analyzer.ProblemAnalyzer(prb)

# anal.print()
# anal.printVariables('f_ball_1')
# anal.printVariables('f_ball_2')
# anal.printVariables('f_ball_3')
# anal.printVariables('f_ball_4')

# anal.printConstraints('zero_velocity_ball_1_ball_1_vel_cartesian_task')
# anal.printConstraints('zero_velocity_ball_2_ball_2_vel_cartesian_task')
# anal.printConstraints('zero_velocity_ball_3_ball_3_vel_cartesian_task')
# anal.printConstraints('zero_velocity_ball_4_ball_4_vel_cartesian_task')


ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

# for name, element in solution.items():
#     logger.create(name, element.shape[0])

# logger.create('xig', solution['x_opt'].shape[0])
# logger.create('xig_robot', solution['x_opt'].shape[0])
# logger.create('q_res', solution['q'].shape[0])

iteration = 0
rate = rospy.Rate(1 / dt)

contact_list_repl = list(contact_dict.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)
                                           # future_trajectory_markers={'base_link': 'world', 'ball_1': 'world'})

base_weight = 0.1
global joy_msg

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()

from joy_commands import GaitManager, JoyCommands
contact_phase_map = {c: f'{c}_timeline' for c in contact_dict}
gm = GaitManager(ti, pm, contact_phase_map)

jc = JoyCommands(gm)


while not rospy.is_shutdown():
    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]

    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    # u_opt = solution['u_opt']
    # uig = np.roll(u_opt, shift_num, axis=1)
    # for i in range(abs(shift_num)):
    #     uig[:, -1 - i] = u_opt[:, -1]
    #
    # prb.getInput().setInitialGuess(uig)

    # shift phases of phase manager
    tic = time.time()
    pm._shift_phases()
    time_elapsed_shifting = time.time() - tic
    time_elapsed_shifting_list.append(time_elapsed_shifting)

    jc.run(solution)

    iteration = iteration + 1

    # solve real time iteration
    # anal.printVariables('f_ball_1', suppress_ig=True)
    # anal.printVariables('f_ball_2', suppress_ig=True)
    # anal.printVariables('f_ball_3', suppress_ig=True)
    # anal.printVariables('f_ball_4', suppress_ig=True)

    # anal.printConstraints('zero_velocity_ball_1_ball_1_vel_cartesian_task')
    # anal.printConstraints('zero_velocity_ball_2_ball_2_vel_cartesian_task')
    # anal.printConstraints('zero_velocity_ball_3_ball_3_vel_cartesian_task')
    # anal.printConstraints('zero_velocity_ball_4_ball_4_vel_cartesian_task')

    # anal.printConstraints('', suppress_ig=True)

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
