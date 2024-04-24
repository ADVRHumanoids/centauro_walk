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
import phase_manager.pyrosserver as pyrosserver

from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS

from joy_commands import JoyCommands

import cartesian_interface.roscpp_utils as roscpp
import cartesian_interface.pyci as pyci
import cartesian_interface.affine3
import horizon.utils.analyzer as analyzer

from base_estimation.msg import ContactWrenches
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from scipy.spatial.transform import Rotation

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from kyon_controller.msg import WBTrajectory

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
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

rospy.init_node('centauro_walk_srbd')
roscpp.init('centauro_walk_srbd', [])

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
ns = 40
T = 2.
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

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
joystick interface
'''
joystick_flag = rospy.get_param(param_name='~joy', default=False)

'''
open/closed loop
'''
closed_loop = rospy.get_param(param_name='~closed_loop', default=False)

'''
xbot
'''
xbot_param = rospy.get_param(param_name="~xbot", default=False)

base_pose = None
base_twist = None
# est = None
robot = None

if xbot_param:
    robot = xbot.RobotInterface(cfg)
    robot.sense()

    if not closed_loop:
        rospy.Subscriber('/xbotcore/imu/imu_link', Imu, imu_callback)
        while base_pose is None:
            rospy.sleep(0.01)

        base_pose[0:3] = [0.07, 0., 0.8]
        base_twist = np.zeros(6)
    else:
        # rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)
        # rospy.Subscriber('/xbotcore/link_state/pelvis/twist', TwistStamped, gt_twist_callback)
        rospy.Subscriber('/centauro_base_estimation/base_link/pose', PoseStamped, gt_pose_callback)
        rospy.Subscriber('/centauro_base_estimation/base_link/twist', TwistStamped, gt_twist_callback)

        while base_pose is None or base_twist is None:
            rospy.sleep(0.01)

    q_init = robot.getJointPosition()
    q_init = robot.eigenToMap(q_init)

else:
    print('RobotInterface not created')

    q_init = {
        # 'torso_yaw': 0.00,  # 0.00,
        # 'j_arm1_1': 1.50,  # 1.60,
        # 'j_arm1_2': 0.1,  # 0.,
        # 'j_arm1_3': 0.2,  # 1.5,
        # 'j_arm1_4': -2.2,  # 0.3,
        # 'j_arm1_5': 0.00,  # 0.00,
        # 'j_arm1_6': -1.3,  # 0.,
        # 'j_arm1_7': 0.0,    # 0.0,

        # 'j_arm2_1': 1.50,  # 1.60,
        # 'j_arm2_2': 0.1,  # 0.,
        # 'j_arm2_3': -0.2,  # 1.5,
        # 'j_arm2_4': -2.2,  # -0.3,
        # 'j_arm2_5': 0.0,  # 0.0,
        # 'j_arm2_6': -1.3,  # 0.,
        # 'j_arm2_7': 0.0,    # 0.0,

        # 'd435_head_joint': 0.0,
        # 'velodyne_joint': 0.0,
        'hip_yaw_1': -0.746,
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
        'ankle_pitch_4': -0.3,
    }

    base_pose = np.array([0.07, 0., 0.8, 0., 0., 0., 1.])
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

# replace continuous joints with revolute
urdf = urdf.replace('continuous', 'revolute')

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joint_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 fixed_joint_map=fixed_joint_map
                                 )

rospy.set_param('mpc/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=mpc/robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/centauro_config.yaml')

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns)

# phase manager handling
c_timelines = dict()
for c in model.cmap.keys():
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')


short_stance_duration = 5
stance_duration = 15
flight_duration = 15
c_i = 0

# for c in model.getContactMap():
#     c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
#     prb.createResidual(f'{c}_ori', c_ori.T - np.array([0, 0, 1]))

for c in model.getContactMap():
    c_i += 1  # because contact task start from contact_1
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
    if ti.getTask(f'contact_{c_i}') is not None:
        stance_phase.addItem(ti.getTask(f'contact_{c_i}'))
        stance_phase_short.addItem(ti.getTask(f'contact_{c_i}'))
    else:
        raise Exception('task not found')

    # flight phase normal
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ee_vel = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot + 0.01, 0.1, [None, 0, None]))
    if ti.getTask(f'z_contact_{c_i}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_contact_{c_i}'), ref_trj)
    else:
        raise Exception('task not found')

    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

    c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
    cost_ori = prb.createResidual(f'{c}_ori', 5. * (c_ori.T - np.array([0, 0, 1])))
    flight_phase.addCost(cost_ori)

for c in model.cmap.keys():
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)

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

rs = pyrosserver.RosServerClass(pm)
def dont_print(*args, **kwargs):
    pass
ti.solver_rti.set_iteration_callback(dont_print)

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl,
                                           fixed_joint_map=fixed_joint_map,
                                           future_trajectory_markers=['contact_1', 'contact_2', 'contact_3', 'contact_4'])

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()
time_elapsed_solving_list = list()
time_elapsed_all_list = list()

contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

if joystick_flag:
    from joy_commands import JoyCommands
    jc = JoyCommands()

gait_manager_ros = GaitManagerROS(gm)

robot_joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

q_robot = np.zeros(len(robot_joint_names))
qdot_robot = np.zeros(len(robot_joint_names))

wrench_pub = rospy.Publisher('centauro_base_estimation/contacts/set_wrench', ContactWrenches, latch=False)


from geometry_msgs.msg import PointStamped
zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)
# zmp_f = ti.getTask('zmp')._zmp_fun()
zmp_point = PointStamped()

c_mean_pub = rospy.Publisher('c_mean_pub', PointStamped, queue_size=10)
c_mean_point = PointStamped()


while not rospy.is_shutdown():
    # update BaseEstimation
    wrench_msg = ContactWrenches()
    wrench_msg.header.stamp = rospy.Time.now()
    for frame in model.getForceMap():
        wrench_msg.names.append(frame)
        wrench_msg.wrenches.append(Wrench(force=Vector3(x=solution[f'f_{frame}'][0, 0],
                                                        y=solution[f'f_{frame}'][1, 0],
                                                        z=solution[f'f_{frame}'][2, 0]),
                                          torque=Vector3(x=0., y=0., z=0.)))

    t0 = time.time()
    wrench_pub.publish(wrench_msg)

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

    pm.shift()

    # publishes to ros phase manager info
    rs.run()

    if joystick_flag:
        # receive msgs from joystick and publishes to ROS topic
        jc.run()

    # receive msgs from ros topic and send commands to robot
    gait_manager_ros.run()

    ti.rti()
    solution = ti.solution

    sol_msg = WBTrajectory()
    sol_msg.header.frame_id = 'world'
    sol_msg.header.stamp = rospy.Time.now()

    sol_msg.joint_names = robot_joint_names

    sol_msg.q = solution['q'][:, 0].tolist()
    sol_msg.v = solution['v'][:, 0].tolist()
    sol_msg.a = solution['a'][:, 0].tolist()

    for frame in model.getForceMap():
        sol_msg.force_names.append(frame)
        sol_msg.f.append(
            Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))

    solution_publisher.publish(sol_msg)

    # replay stuff

    # =========================== publish zmp =================================================
    # input_zmp = []
    # input_zmp.append(solution['q'][:, 0])
    # input_zmp.append(solution['v'][:, 0])
    # input_zmp.append(solution['a'][:, 0])
    #
    # for f_var in model.fmap.keys():
    #     input_zmp.append(solution[f"f_{f_var}"][:, 0])
    #
    # c_mean = np.zeros([3, 1])
    # f_tot = np.zeros([3, 1])
    # for c_name, f_var in model.fmap.items():
    #     fk_c_pos = kin_dyn.fk(c_name)(q=solution['q'][:, 0])['ee_pos'].toarray()
    #     c_mean += fk_c_pos * solution[f"f_{c_name}"][2, 0]
    #     f_tot += solution[f"f_{c_name}"][2, 0]
    #
    # print(f_tot[2, 0])
    # c_mean /= f_tot
    #
    # # zmp_val = zmp_f(*input_zmp)
    # zmp_val = zmp(model)(*input_zmp)
    #
    # zmp_point.header.stamp = rospy.Time.now()
    # zmp_point.header.frame_id = 'world'
    # zmp_point.point.x = zmp_val[0]
    # zmp_point.point.y = zmp_val[1]
    # zmp_point.point.z = 0
    #
    # zmp_pub.publish(zmp_point)
    #
    # c_mean_point.header.stamp = rospy.Time.now()
    # c_mean_point.header.frame_id = 'world'
    # c_mean_point.point.x = c_mean[0]
    # c_mean_point.point.y = c_mean[1]
    # c_mean_point.point.z = 0
    #
    # c_mean_pub.publish(c_mean_point)
    # ============================================================================

    if robot is None:
        repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
        repl.publish_joints(solution['q'][:, 0])
        repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
        repl.publish_future_trajectory_marker(solution['q'])

    solution_time_publisher.publish(Float64(data=time.time() - t0))
    rate.sleep()


# print(f'average time elapsed solving: {sum(time_elapsed_all_list) / len(time_elapsed_all_list)}')
