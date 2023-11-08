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
from multiprocessing import Process


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

    initial_time = 0.1
    final_time = 0.1
    cycle_time = 1.6
    n_cycles = 4

    tf = n_cycles * cycle_time + initial_time + final_time

    dt = cycle_time * 1/5 /4
    ns = int(tf/dt)

    cycle_duration = int(cycle_time / dt)
    init_nodes = int(initial_time / dt)
    final_nodes = int(final_time / dt)
    end_nodes = ns - final_nodes

    final_x_displacement = 2.

    print('n_nodes: ', ns)
    print('dt: ', dt)
    print(f'n_cycles: {n_cycles}')
    print('tf:', tf)

    print('mass:', kd.mass())

    prb = Problem(ns)
    prb.setDt(dt)
    # set up model
    model = FullModelInverseDynamics(problem=prb,
                                     kd=kd,
                                     q_init=q_init,
                                     base_init=base_init)

    for contact in contacts:
        model.setContactFrame(contact, 'vertex', dict(vertex_frames=[contact]))

    # contacts = list(model.fmap.keys())
    # model.v.setBounds(-1 * kd.velocityLimits(), kd.velocityLimits())

    fin_q = model.q0.copy()
    fin_q[0] = fin_q[0] + final_x_displacement

    # start-stop
    model.q.setBounds(model.q0, model.q0, 0)
    model.q[0].setBounds(fin_q[0], fin_q[0], ns)

    v_init = np.zeros(model.nv)
    model.v.setBounds(v_init, v_init, 0)
    model.v.setBounds(v_init, v_init, ns)

    model.q.setInitialGuess(model.q0)

    for f_name, f_var in model.fmap.items():
        f_var.setInitialGuess([0, 0, kd.mass() / 4 * 9.8])


    contact_pos = dict()



    duty_cycle = 1 - (1/len(contacts))
    single_cycle = int(cycle_duration / len(contacts))

    print(f'cycle duration: {cycle_duration * dt} ({cycle_duration} nodes)')
    print(f'single contact cycle: {single_cycle * dt} ({single_cycle} nodes)')


    phi_vec = [0.25 , 0.75, 0., 0.5]

    # print(contacts)
    # exit()
    # phi_phase = 1/len(contacts)
    # for i in range(len(contacts)):
    #     phi_vec.append(i * phi_phase)

    pg = PatternGenerator(ns, contacts)



    # prepare the pattern containers
    stance_nodes_rep = dict()
    swing_nodes_rep = dict()
    for contact in contacts:
        stance_nodes_rep[contact] = []
        swing_nodes_rep[contact] = []

    # repeat pattern for n cycles
    last_n = 0
    for n in range(0, n_cycles):

        if n == 1:
            current_cycle_duration = int(cycle_duration)
        else:
            current_cycle_duration = int(cycle_duration)

        stance_nodes, swing_nodes = pg.generateCycle(phi_vec, current_cycle_duration, duty_cycle=duty_cycle)

        # pg.visualizer(current_cycle_duration, stance_nodes, swing_nodes)

        for name, value in swing_nodes.items():
            swing_nodes_rep[name].extend([elem + last_n for elem in value])

        for name, value in stance_nodes.items():
            stance_nodes_rep[name].extend([elem + last_n for elem in value])

        last_n += current_cycle_duration

    # add initial nodes
    for name, value in swing_nodes_rep.items():
        swing_nodes_rep[name] = [x + init_nodes for x in value]

    for name, value in stance_nodes_rep.items():
        stance_nodes_rep[name] = [x + init_nodes for x in value]


    # remove nodes if they are outside range
    for name, value in swing_nodes_rep.items():
        swing_nodes_rep[name] = [x for x in value if x < end_nodes]
    #
    for name, value in stance_nodes_rep.items():
        stance_nodes_rep[name] = [x for x in value if x < end_nodes]


    for name, value in stance_nodes_rep.items():
        stance_nodes_rep[name].extend(list(range(0, init_nodes)))

    for name, value in stance_nodes_rep.items():
        stance_nodes_rep[name].extend(list(range(end_nodes, ns)))

    print('stance_nodes:')
    for name, nodes in stance_nodes_rep.items():
        print(f'{name}:, {nodes}')
    print('swing_nodes:')
    for name, nodes in swing_nodes_rep.items():
        print(f'{name}:, {nodes}')

    # pg.visualizer(cycle_duration * n_cycles + init_nodes + final_nodes, stance_nodes_rep, swing_nodes_rep)

    swing_nodes = swing_nodes_rep.copy()
    stance_nodes = stance_nodes_rep.copy()

    z_des = dict()
    clea = dict()
    x_pos_cnsrt = dict()
    # contact velocity is zero, and normal force is positive

    # x-pos for gap crossing
    orig = 1.
    lims = 0.1  # 0.25   #0.1#
    slope = 20  # 5  # 10

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



        exp1 = 1/np.exp(-slope*(p[0]-orig-lims))
        exp2 = 1/np.exp(slope*(p[0]-orig+lims))
        f_mountain = 1 / (exp1 + exp2)

        x_pos_cnsrt[frame] = prb.createResidual(f"{frame}_x_pos", f_mountain)

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

    prb.createResidual("min_q", 0.08 * (model.q[postural_joints] - model.q0[postural_joints]))
    if white_list:
        prb.createResidual("min_q_white_list", 0.4 * (model.q[white_list_indices] - model.q0[white_list_indices]))

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
        z_trj = np.atleast_2d(tg.from_derivatives(single_cycle, pos_z, pos_z, clearance, [0, 0, 0]))
        rep_param = np.concatenate([z_trj] * int((len(swing_nodes[c]) + 10) / z_trj.shape[1]), axis=1)
        z_des[c].assign(rep_param[:, :len(swing_nodes[c])], nodes=swing_nodes[c])
        print(z_trj)

        # x_values = np.atleast_2d(x_trj[c])
        # x_des[c].assign(x_values[:, :len(swing_nodes[c])], nodes=swing_nodes[c])
        # print(x_values)

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
            'ipopt.tol': 1e-3,
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

    from marker_spawner import make_gap
    import functools
    callable_gap = functools.partial(make_gap, gap_center=orig, gap_width=lims*2)
    p = Process(target=callable_gap)
    p.start()

    return prb, solution






