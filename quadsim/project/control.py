from typing import Tuple
import numpy as np


def control(
    # Time elapsed in the simulation [s]
    t: float,

    # Robot base linear velocity [m/s]
    # [x, y, z], shape=(3,)
    v: np.ndarray,

    # Robot base angular velocity [rad/s]
    # [x, y, z], shape=(3,)
    omega: np.ndarray,

    # Robot base orientation in quaternion [-]
    # [x, y, z, w], shape=(4,)
    quat: np.ndarray,

    # Joint position [rad]
    # [front_right_hip, front_right_thigh, front_right_calf,
    #   front_left_hip,  front_left_thigh,  front_left_calf,
    #   rear_right_hip,  rear_right_thigh,  rear_right_calf,
    #    rear_left_hip,   rear_left_thigh,   rear_left_calf], shape=(12,)
    q: np.ndarray,

    # Joint velocity [rad/s]
    # Order and dimension same as q
    dq: np.ndarray,

    # Robot foot contact [-] (1 stands for contact, 0 non-contact)
    # [front_right, front_left, rear_right, rear_left], shape=(4,)
    foot_contact: np.ndarray
) -> Tuple[
    # Output (joint torque [Nm] or position [rad])
    # Order and dimension same as q
    np.ndarray,

    # If use torque output (otherwise use position)
    bool,

    # Desired joint kp values (valid only if use position control)
    # Order and dimension same as q
    np.ndarray,

    # Desired joint kd values (valid only if use position control)
    # Order and dimension same as q
    np.ndarray
]:
    '''Position control example'''
    # Create a dummy output with zero positions
    out = np.zeros(12)
    # Set to use position control
    use_torque = False
    # Create a dummy kp with zeros
    kp = np.zeros(12)
    # Create a dummy kd with zeros
    kd = np.zeros(12)

    '''Torque control example'''
    # Create a dummy output with zero torques
    out = np.zeros(12)
    # Set to use torque control
    use_torque = True
    # Create a placeholder for kp
    kp = np.empty(12)
    # Create a placeholder for kd
    kd = np.empty(12)

    '''Assemble and return the control'''
    return (out, use_torque, kp, kd)
