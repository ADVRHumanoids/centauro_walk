from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.problem import Problem
from horizon.solvers import Solver
from casadi_kin_dyn import pycasadi_kin_dyn
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.patternGenerator import PatternGenerator
from horizon.utils.trajectoryGenerator import TrajectoryGenerator
from horizon.utils import utils as horizon_utils, kin_dyn
import time, rospy
import numpy as np
import copy


def run(q_init,
        base_init,
        contacts,
        solver_type,
        kd,
        transcription_method,
        transcription_opts=None,
        flag_upper_body=True,
        kd_frame=pycasadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED,
        ):

    ns = 50  # trot
    tf = 4.  # 10s
    v = 0.3

    dt = tf / ns
    print('dt: ', dt)

    prb = Problem(ns)
    prb.setDt(dt)
    # set up model
    model = FullModelInverseDynamics(problem=prb,
                                     kd=kd,
                                     q_init=q_init,
                                     base_init=base_init)

    for contact in contacts:
        model.setContactFrame(contact, 'vertex', dict(vertex_frames=[contact]))

    fin_q = model.q0.copy()
    fin_q[0] = fin_q[0] + tf*v

    model.q.setBounds(model.q0, model.q0, 0)
    # model.q[0].setBounds(fin_q[0], fin_q[0], ns)
    model.q.setBounds(fin_q, fin_q, ns)

    v_init = np.zeros(model.nv)
    model.v.setBounds(v_init, v_init, 0)
    model.v.setBounds(v_init, v_init, ns)

    model.q.setInitialGuess(model.q0)

    for f_name, f_var in model.fmap.items():
        f_var.setInitialGuess([0, 0, kd.mass() / 4 * 9.8])

    gait_matrix = np.array([[1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1],
                            [0, 1, 0, 0]]).astype(int)

    contact_pos = dict()

    cycle_duration = 20 #int(ns / tf * 1.5)
    print(f'cycle duration: {cycle_duration}')
    duty_cycle = 1.
    flight_with_duty = int(cycle_duration / gait_matrix.shape[1] * duty_cycle)
    n_init_nodes = 4

    pg = PatternGenerator(ns - n_init_nodes, contacts)
    stance_nodes, swing_nodes, cycle_duration = pg.generateCycle_old(gait_matrix, cycle_duration, duty_cycle=duty_cycle)

    n_cycles = int((ns - n_init_nodes) / cycle_duration) - 1
    for contact in contacts:
        i = cycle_duration
        swing_nodes_temp = swing_nodes[contact].copy()
        stance_nodes_temp = stance_nodes[contact].copy()
        for cycle in range(n_cycles):
            swing_nodes[contact].extend([i + elem for elem in swing_nodes_temp])
            stance_nodes[contact].extend([i + elem for elem in stance_nodes_temp])
            i += cycle_duration

    for key, value in stance_nodes.items():
        stance_nodes[key] = [elem + n_init_nodes for elem in value]
    for key, value in swing_nodes.items():
        swing_nodes[key] = [elem + n_init_nodes for elem in value]
    for contact in contacts:
        for x in range(n_init_nodes - 1, -1, -1):
            stance_nodes[contact].insert(0, x)

    for contact in contacts:
        [stance_nodes[contact].append(i) for i in range(swing_nodes[contact][-1] + 1, ns)]

    print('stance_nodes:')
    for name, nodes in stance_nodes.items():
        print(f'{name}:, {nodes}')
    print('swing_nodes:')
    for name, nodes in swing_nodes.items():
        print(f'{name}:, {nodes}')

    z_des = dict()
    clea = dict()
    # contact velocity is zero, and normal force is positive
    for i, frame in enumerate(contacts):
        FK = kd.fk(frame)
        DFK = kd.frameVelocity(frame, kd_frame)

        p = FK(q=model.q)['ee_pos']
        v = DFK(q=model.q, qdot=model.v)['ee_vel_linear']

        # kinematic contact
        fcost = horizon_utils.barrier(model.fmap[frame][2] - 10.0)  # fz > 10
        stance_nodes_vel = copy.deepcopy(stance_nodes)
        for k in contacts:
            for j in range(1, len(stance_nodes_vel[k])):
                if stance_nodes_vel[k][j] != stance_nodes_vel[k][j - 1] + 1:
                    stance_nodes_vel[k].insert(j, stance_nodes_vel[k][j] - 1)
        prb.createConstraint(f"{frame}_vel", v, nodes=stance_nodes_vel[frame])

        # unilateral forces
        prb.createIntermediateCost(f'{frame}_unil', 1e1 * fcost, nodes=stance_nodes[frame])


        # clearance
        contact_pos[frame] = FK(q=model.q0)['ee_pos']
        z_des[frame] = prb.createParameter(f'{frame}_z_des', 1)
        clea[frame] = prb.createConstraint(f"{frame}_clea", p[2] - z_des[frame], nodes=swing_nodes[frame])

        if swing_nodes[frame]:
            model.fmap[frame].setBounds(np.array([[0, 0, 0]] * len(swing_nodes[frame])).T,
                                        np.array([[0, 0, 0]] * len(swing_nodes[frame])).T,
                                        nodes=swing_nodes[frame])

    # joint posture
    black_list_indices = list()
    white_list_indices = list()
    black_list = []
    white_list = []
    if flag_upper_body:
        black_list = ['shoulder_yaw_1', 'shoulder_pitch_1', 'elbow_pitch_1', 'shoulder_yaw_2', 'shoulder_pitch_2', 'elbow_pitch_2']
        white_list = ['shoulder_yaw_1', 'shoulder_pitch_1', 'elbow_pitch_1', 'shoulder_yaw_2', 'shoulder_pitch_2', 'elbow_pitch_2']

    postural_joints = np.array(list(range(7, model.nq)))
    for joint in black_list:
        black_list_indices.append(model.joint_names.index(joint))
    for joint in white_list:
        white_list_indices.append(7 + model.joint_names.index(joint))
    postural_joints = np.delete(postural_joints, black_list_indices)

    prb.createResidual("min_q", 0.1 * (model.q[postural_joints] - model.q0[postural_joints]))
    if white_list:
        prb.createResidual("min_q_white_list", 0.5 * (model.q[white_list_indices] - model.q0[white_list_indices]))

    # joint acceleration
    prb.createIntermediateResidual("min_q_ddot", 0.001 * model.a)

    # contact forces
    for f_name, f_var in model.fmap.items():
        prb.createIntermediateResidual(f"min_{f_var.getName()}", 0.001 * f_var)


    # z axis reference trajectory
    tg = TrajectoryGenerator()
    clearance = 0.1
    for c in contacts:
        pos_z = contact_pos[c][2].elements()[0]
        z_trj = np.atleast_2d(tg.from_derivatives(flight_with_duty, pos_z, pos_z, clearance, [0, 0, 0]))
        rep_param = np.concatenate([z_trj] * int((len(swing_nodes[c]) + 10) / z_trj.shape[1]), axis=1)
        z_des[c].assign(rep_param[:, :len(swing_nodes[c])], nodes=swing_nodes[c])
        print(z_trj)

    model.setDynamics()

    # motor
    mm2Tom2 = 1e-6
    I = np.diag([9.9994591e2 * mm2Tom2] * (model.nq - 7))
    k = 0.129
    gear_ratio = 1. / 30.
    efficiency = 0.7
    # motor_dyn = prb.createIntermediateConstraint("motor_dyn",
    #                                              (I @ model.a[6:] / gear_ratio + model.tau[6:] * gear_ratio / efficiency) / k)
    # motor_dyn.setBounds(np.array([-40] * (model.nq - 7)), np.array([40] * (model.nq - 7)))

    # torque minimization
    # prb.createIntermediateResidual('min_tau', 0.01 * model.tau)

    if solver_type != 'ilqr':
        Transcriptor.make_method(transcription_method, prb, transcription_opts)

    opts = {'ipopt.max_iter': 200,
            'ipopt.tol': 1e-4,
            'ipopt.constr_viol_tol': 1e-3,
            'ilqr.max_iter': 200,
            'ilqr.alpha_min': 0.01,
            'ilqr.step_length_threshold': 1e-9,
            'ilqr.line_search_accept_ratio': 1e-4,
            }

    solver_bs = Solver.make_solver(solver_type, prb, opts)

    try:
        solver_bs.set_iteration_callback()
    except:
        pass

    t = time.time()
    if not solver_bs.solve():
        raise RuntimeError('kitemmurt')

    elapsed = time.time() - t
    print(f'bootstrap solved in {elapsed} s')

    solution = solver_bs.getSolutionDict()

    # append torques to solution
    tau = list()
    id_fn = kin_dyn.InverseDynamics(kd, contacts, kd_frame)
    for i in range(solution['q'].shape[1] - 1):
        tau.append(id_fn.call(solution['q'][:, i], solution['v'][:, i], solution['a'][:, i], {name: solution['f_'+name][:, i] for name in model.fmap}))

    current = list()
    for i in range(solution['q'].shape[1] - 1):
        current.append((I @ solution['a'][6:, i] / gear_ratio + tau[i][6:] * gear_ratio / efficiency) / k)

    solution['tau'] = tau
    solution['current'] = current

    return prb, solution






