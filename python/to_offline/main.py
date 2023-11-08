import argparse
import rospkg, rospy
from horizon.utils import mat_storer
from casadi_kin_dyn import pycasadi_kin_dyn
import subprocess
from pathlib import Path
import numpy as np
from horizon.ros import replay_trajectory

def init_problem():

    # set up problem
    contacts = ['ball_1', 'ball_2', 'ball_3', 'ball_4']

    # set up solver
    solver_type = 'ipopt'

    transcription_method = 'multiple_shooting'
    transcription_opts = dict(integrator='RK4')

    # get path of kyon urdf in the system
    kyon_urdf_folder = rospkg.RosPack().get_path('kyon_urdf')

    flag_upper_body = False
    # set up model
    urdf = subprocess.check_output(["xacro", kyon_urdf_folder + "/urdf/kyon.urdf.xacro",
                                    "sensors:=false",
                                    f"upper_body:={flag_upper_body}",
                                    ])
    urdf = urdf.decode('utf-8')
    rospy.set_param('/robot_description', urdf)

    # spawn process with rosrun robot_state_publisher
    bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
    subprocess.Popen(bashCommand.split(), start_new_session=True)

    # use casadi_kin_dyn to parse the urdf and convert in a Horizon-compatible object
    kd = pycasadi_kin_dyn.CasadiKinDyn(urdf)

    hip_pitch = 0.7
    knee_pitch = -2 * hip_pitch

    base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    q_init = {'hip_roll_1': 0.0,
              'hip_pitch_1': hip_pitch,
              'knee_pitch_1': knee_pitch,

              'hip_roll_2': 0.0,
              'hip_pitch_2': hip_pitch,
              'knee_pitch_2': knee_pitch,

              'hip_roll_3': 0.0,
              'hip_pitch_3': hip_pitch,
              'knee_pitch_3': knee_pitch,

              'hip_roll_4': 0.0,
              'hip_pitch_4': hip_pitch,
              'knee_pitch_4': knee_pitch,
              }

    if flag_upper_body:

        q_init.update({'shoulder_yaw_1': 0.0,
                       'shoulder_pitch_1': -np.pi/2,
                       'elbow_pitch_1': 0.0,
                       'wrist_pitch_1': 0.,
                       'wrist_yaw_1': 0.,

                       'shoulder_yaw_2': 0.0,
                       'shoulder_pitch_2': -np.pi/2,
                       'elbow_pitch_2': 0.0,
                       'wrist_pitch_2': 0.,
                       'wrist_yaw_2': 0.})


        # q_init.update({'shoulder_yaw_1': 0.0,
        #                'shoulder_pitch_1': 0.9,
        #                'elbow_pitch_1': 1.68,
        #                'wrist_pitch_1': 0.,
        #                'wrist_yaw_1': 0.,
        #
        #                'shoulder_yaw_2': 0.0,
        #                'shoulder_pitch_2': 0.9,
        #                'elbow_pitch_2': 1.68,
        #                'wrist_pitch_2': 0.,
        #                'wrist_yaw_2': 0.})

    FK = kd.fk('ball_1')
    init = base_init.tolist() + list(q_init.values())
    init_pos_foot = FK(q=init)['ee_pos']
    base_init[2] = -init_pos_foot[2]

    return q_init, base_init, solver_type, kd, transcription_method, transcription_opts, contacts, flag_upper_body

def main(args):

    action = args.action

    q_init, base_init, solver_type, kd, transcription_method, transcription_opts, contacts, flag_upper_body = init_problem()

    if action == 'step_up':
        import _step_up
        prb, solution = _step_up.run(q_init, base_init, contacts, solver_type, kd, transcription_method, transcription_opts, flag_upper_body)
    elif action == 'trot':
        import _trot
        prb, solution = _trot.run(q_init, base_init, contacts, solver_type, kd, transcription_method, transcription_opts, flag_upper_body)
    elif action == 'crawl':
        import _crawl
        prb, solution = _crawl.run(q_init, base_init, contacts, solver_type, kd, transcription_method, transcription_opts, flag_upper_body)
    elif action == 'gap':
        import _gap
        prb, solution = _gap.run(q_init, base_init, contacts, solver_type, kd, transcription_method, transcription_opts, flag_upper_body)
    elif action == 'gaits':
        import _gaits
        prb, solution = _gaits.run(q_init, base_init, contacts, solver_type, kd, transcription_method, transcription_opts)
    else:
        raise Exception('wrong action commanded')

        # =========================================================================

    # if args.filename:
    #     if not os.path.exists("./mat_files"):
    #         os.mkdir("./mat_files")
    #     ms = mat_storer.matStorer('mat_files/' + args.filename + '.mat')
    #     info_dict = dict(n_nodes=prb.getNNodes(), dt=prb.getDt())
    #     ms.store({**solution, **info_dict, **kin_params})

    repl = replay_trajectory.replay_trajectory(prb.getDt(), kd.joint_names(), solution['q'], kindyn=kd, trajectory_markers=contacts)
    repl.replay(is_floating_base=True)


if __name__ == '__main__':
    actions = ('trot', 'step_up', 'jump_up', 'crawl')
    # solvers = ('ipopt', 'ilqr', 'gnsqp')

    parser = argparse.ArgumentParser(
        description='todo')
    parser.add_argument('--action', '-a', help='choose which action spot will perform', choices=actions,
                        default=actions[1])
    parser.add_argument('--filename', help='name of the storer file')
    # parser.add_argument('--solver', '-s', help='choose which solver will be used', choices=spot_solvers,
    #                     default=spot_solvers[0])
    # parser.add_argument('--replay', '-r', help='visualize the robot trajectory in rviz', action='store_true',
    #                     default=False)
    # parser.add_argument("--codegen", '-c', type=str2bool, nargs='?', const=True, default=False,
    #                     help="generate c++ code for faster solving")
    # parser.add_argument("--warmstart", '-w', type=str2bool, nargs='?', const=True, default=False,
    #                     help="save solutions to mat file")
    # parser.add_argument("--plot", '-p', type=str2bool, nargs='?', const=True, default=True, help="plot solutions")

    args = parser.parse_args()

    # args.action = 'trot'
    args.action = 'gap'
    # args.action = 'gaits'
    # args.action = 'crawl'
    # args.action = 'step_up'
    main(args)