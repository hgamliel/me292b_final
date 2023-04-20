import numpy as np
from scipy.io import savemat
import time
from math import sqrt
from tqdm import tqdm
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from quadsim.envs import env_builder
from quadsim.robots import robot_config


USE_REAL_ROBOT: bool = False
USE_MATLAB: bool = True
DT: float = .002  # [s]
SIM_DURATION: float = 2.  # [s]
DEFUALT_JOINT_POS: np.ndarray = np.array([
    -.1, .8, -1.5,
    .1, .8, -1.5,
    -.1, .8, -1.5,
    .1, .8, -1.5
])
TORQUE_LIMIT: np.ndarray = np.array([
    20., 55., 55.,
    20., 55., 55.,
    20., 55., 55.,
    20., 55., 55.
])


if USE_REAL_ROBOT:
    from quadsim.robots import a1_robot
else:
    from quadsim.robots import a1

if USE_MATLAB:
    print('Loading MATLAB engine...')
    import matlab.engine
    eng = matlab.engine.start_matlab()
    eng.cd(r'.', nargout=0)

    def control(t, v, omega, quat, q, dq, foot_contact):
        out = eng.control(t, v, omega, quat, q, dq, foot_contact, nargout=4)
        return (np.asarray(out[0])[0], out[1],
                np.asarray(out[2])[0], np.asarray(out[3])[0])
    print('Finished loading MATLAB engine.')
else:
    import control
    control = control.control

if USE_REAL_ROBOT:
    input("Press [Enter] to continue.")

if USE_MATLAB:
    desc = 'Controlling with MATLAB func'
else:
    desc = 'Controlling with Py func'

# Create a robot instance according to the robot type
if USE_REAL_ROBOT:
    p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot = a1_robot.A1Robot(
        pybullet_client=p,
        motor_control_mode=robot_config.MotorControlMode.HYBRID,
        enable_action_interpolation=False,
        time_step=DT,
        action_repeat=1
    )
else:
    env = env_builder.build_regular_env(
        robot_class=a1.A1,
        motor_control_mode=robot_config.MotorControlMode.HYBRID,
        enable_rendering=True,
        on_rack=False,
        wrap_trajectory_generator=False
    )
    env.set_time_step(1, sim_step=DT)
    robot = env.robot
    zero_command = np.zeros(12)
    for t in range(120):
        robot.Step(zero_command, robot_config.MotorControlMode.TORQUE)

# Move the motors slowly to the initial stand-up configuration
robot.ReceiveObservation()
current_motor_angle = robot.GetMotorAngles()
for t in tqdm(range(400), desc='Lifting up the robot'):
    time.sleep(.002)
    robot.ReceiveObservation()
    blend_ratio = np.minimum(t / 300., 1.)
    out = (1 - blend_ratio) * current_motor_angle + \
        blend_ratio * DEFUALT_JOINT_POS
    robot.Step(out, robot_config.MotorControlMode.POSITION)

# Data buffers
if not USE_REAL_ROBOT:
    distance = 0.
    score = 10. + 65. * min(distance, 1.)
    t_rec = np.ones([0, 1], dtype=float)
    v_rec = np.ones([0, 3], dtype=float)
    omega_rec = np.ones([0, 3], dtype=float)
    quat_rec = np.ones([0, 4], dtype=float)
    q_rec = np.ones([0, 12], dtype=float)
    dq_rec = np.ones([0, 12], dtype=float)
    foot_contact_rec = np.ones([0, 4], dtype=float)
    out_rec = np.ones([0, 12], dtype=float)
    kp_rec = np.ones([0, 12], dtype=float)
    kd_rec = np.ones([0, 12], dtype=float)
    tau_rec = np.ones([0, 12], dtype=float)
    pos_rec = np.ones([0, 3], dtype=float)
    score_rec = np.ones([0, 1], dtype=float)

# Execute the controller
for t in tqdm(range(int(SIM_DURATION / DT)), desc=desc):
    # Observation
    robot.ReceiveObservation()
    t = float(DT * t)
    v = np.asarray(robot.GetBaseVelocity())
    omega = robot.GetBaseRollPitchYawRate()
    quat = np.asarray(robot.GetBaseOrientation())
    q = robot.GetMotorAngles()
    dq = robot.GetMotorVelocities()
    foot_contact = np.asarray(robot.GetFootContacts(), dtype=float)

    if not USE_REAL_ROBOT:
        # Update distance score
        px, py, pz = robot.GetBasePosition()
        p = sqrt(px ** 2 + py ** 2)
        distance = max(p, distance)
        score = 10. + 65. * min(distance, 1.)
        pos = np.array([px, py, pz], dtype=float)

        # Log data
        v_rec = np.vstack([v_rec, v])
        omega_rec = np.vstack([omega_rec, omega])
        quat_rec = np.vstack([quat_rec, quat])
        q_rec = np.vstack([q_rec, q])
        dq_rec = np.vstack([dq_rec, dq])
        foot_contact_rec = np.vstack([foot_contact_rec, foot_contact])
        pos_rec = np.vstack([pos_rec, pos])
        score_rec = np.vstack([score_rec, np.array([score], dtype=float)])
        t_rec = np.vstack([t_rec, np.array([t])])

    # Calculate control
    out, use_torque, kp, kd = control(t, v, omega, quat, q, dq, foot_contact)

    # Apply control
    if not use_torque:
        # Calculate the torques from the position commands
        tau = (out - q) * kp - dq * kd
    else:
        tau = out.copy()
    tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
    robot.Step(tau, robot_config.MotorControlMode.TORQUE)

    if not USE_REAL_ROBOT:
        # Log data
        out_rec = np.vstack([out_rec, out])
        kp_rec = np.vstack([kp_rec, kp])
        kd_rec = np.vstack([kd_rec, kd])
        tau_rec = np.vstack([tau_rec, tau])

    if not USE_REAL_ROBOT:
        time.sleep(.0005)
    else:
        time.sleep(DT)

robot.Terminate()

if not USE_REAL_ROBOT:
    np.savez('log.npz',
            t=t_rec,
            v=v_rec,
            omega=omega_rec,
            quat=quat_rec,
            q=q_rec,
            dq=dq_rec,
            foot_contact=foot_contact_rec,
            out=out_rec,
            kp=kp_rec,
            kd=kd_rec,
            tau=tau_rec,
            pos=pos_rec,
            score=score_rec)
    mdic = {'t': t_rec,
            'v': v_rec,
            'omega': omega_rec,
            'quat': quat_rec,
            'q': q_rec,
            'dq': dq_rec,
            'foot_contact': foot_contact_rec,
            'out': out_rec,
            'kp': kp_rec,
            'kd': kd_rec,
            'tau': tau_rec,
            'pos': pos_rec,
            'score': score_rec}
    savemat('log.mat', mdic)
    print('|' * 50)
    print('|' * 50)
    print('\tYour score is %.3f.' % score)
    print('|' * 50)
    print('|' * 50)
