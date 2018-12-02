import numpy as np
import matplotlib.pyplot as plt
from forward_inverse_kinematics import inv_kin
from robot_configuration import L
from visualize import motion_plot


l1 = L[0]
l2 = L[1]
l3 = L[2]

sin = np.sin
cos = np.cos
tan = np.tan

max_freq = 100
max_joint_velocity = 1
max_cartesian_velocity = 200
max_joint_acceleration = 1
max_cartesian_acceleration = 400
junction_steps = 5


# already calculated inverse jacobian
def jacobian_inverse(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    inverse = np.array([[-sin(q1) / (l2 * cos(q2) + l3 * cos(q3)), cos(q1) / (l2 * cos(q2) + l3 * cos(q3)), 0],
                        [-cos(q1) * cos(q3) / (l2 * sin(q2 + q3)), -sin(q1) * cos(q3) / (l2 * sin(q2 + q3)),
                         sin(q3) / (l2 * sin(q2 + q3))],
                        [-cos(q1) * cos(q2) / (l3 * sin(q2 + q3)), -sin(q1) * cos(q2) / (l3 * sin(q2 + q3)),
                         -sin(q2) / (l3 * sin(q2 + q3))]])
    return inverse


def ptp_trajectory(q0, qf):
    dq = qf - q0
    dq_abs = np.abs(dq)
    # dq is a figure area, max_joint_velocity is a height
    t_ba = np.around(dq_abs / max_joint_velocity, 2)
    # t_b is a time for which acceleration > 0 and constant
    t_b = np.around(max_joint_velocity / max_joint_acceleration, 2)

    if np.any(t_b < t_ba):
        # trapezium
        t_a = t_ba - t_b  # t_a is a time of constant velocity
        t_a_max = np.amax(t_a)
        t_f = t_a_max + 2 * t_b

        # we need the velocity plots of each joint to look like trapeziums of equal length
        each_joint_velocity = dq / (t_a_max + t_b)
        each_joint_acceleration = each_joint_velocity / t_b

        time = np.arange(0, t_f+0.005, 0.01)  # add 0.005 to create array with correct size
        v = np.zeros(shape=(dq.shape[0], time.shape[0]))
        pos = np.zeros(shape=(dq.shape[0], time.shape[0]))
        acc = np.zeros(shape=(dq.shape[0], time.shape[0]))

        pos[:, 0] = q0

        for (i,), cur_time in np.ndenumerate(time):
            if cur_time < t_b:
                v[:, i] = each_joint_acceleration * cur_time
            elif cur_time < t_a_max + t_b:
                v[:, i] = each_joint_velocity
            else:
                v[:, i] = each_joint_acceleration * (t_f - cur_time)

        for (i,), cur_time in np.ndenumerate(time):
            if i > 0:
                acc[:, i] = (v[:, i] - v[:, i - 1]) * max_freq
                pos[:, i] = pos[:, i - 1] + (v[:, i] + v[:, i - 1]) / 2 * 0.01

        return pos, v, acc
    else:
        # triangle
        # dq = t_b * v_max
        # t_b = v_max / acc => dq = t_b ^ 2 * acc
        t_b = np.around(np.sqrt(dq_abs/max_joint_acceleration), 2)
        t_b_max = np.amax(t_b)
        t_f = 2 * t_b_max
        each_joint_velocity = dq / t_b_max
        each_joint_acceleration = each_joint_velocity / t_b_max
        time = np.arange(0, t_f + 0.005, 0.01)
        v = np.zeros(shape=(dq.shape[0], time.shape[0]))
        pos = np.zeros(shape=(dq.shape[0], time.shape[0]))
        acc = np.zeros(shape=(dq.shape[0], time.shape[0]))


        pos[:, 0] = q0

        for (i,), cur_time in np.ndenumerate(time):
            if cur_time < t_b_max:
                v[:, i] = each_joint_acceleration * cur_time
            else:
                v[:, i] = each_joint_acceleration * (t_f - cur_time)

        for (i,), cur_time in np.ndenumerate(time):
            if i > 0:
                acc[:, i-1] = (v[:, i] - v[:, i - 1]) * max_freq
                pos[:, i] = pos[:, i - 1] + (v[:, i] + v[:, i - 1]) / 2 * 0.01

        return pos, v, acc


def lin_trajectory(x0, xf):
    dist = np.linalg.norm(xf - x0)
    dist_z = xf[2] - x0[2]
    dist_y = xf[1] - x0[1]
    dist_x = xf[0] - x0[0]
    dist_xy = np.sqrt(dist_x**2 + dist_y**2)
    sin_a = dist_z / dist
    cos_a = dist_xy / dist
    cos_b = dist_x / dist_xy
    sin_b = dist_y / dist_xy

    q0 = inv_kin(x0, L)
    qf = inv_kin(xf, L)

    def get_joint_components(module):
        return np.array([module * cos_a * cos_b, module * cos_a * sin_b, module * sin_a])

    t_ba = np.around(dist / max_cartesian_velocity, 2)
    t_b = np.around(max_cartesian_velocity / max_cartesian_acceleration, 2)

    time = 0
    v_cartesian = 0

    if t_b < t_ba:
        t_a = t_ba - t_b
        t_f = t_ba + t_b
        time = np.arange(0, t_f + 0.005, 1/max_freq)  # add 0.005 to create array with correct size
        v_cartesian = np.zeros(time.shape[0])  # 3 because there are 3 joints

        for (i,), cur_time in np.ndenumerate(time):
            if cur_time < t_b:
                v_cartesian[i] = max_cartesian_acceleration * cur_time
            elif cur_time < t_a + t_b:
                v_cartesian[i] = max_cartesian_velocity
            else:
                v_cartesian[i] = max_cartesian_acceleration * (t_f - cur_time)

    else:
        t_b = np.around(np.sqrt(dist/max_cartesian_acceleration), 2)
        t_f = 2 * t_b
        time = np.arange(0, t_f + 0.005, 1 / max_freq)  # add 0.005 to create array with correct size
        v_cartesian = np.zeros(time.shape[0])  # 3 because there are 3 joints
        for (i,), cur_time in np.ndenumerate(time):
            if cur_time < t_b:
                v_cartesian[i] = max_cartesian_acceleration * cur_time
            else:
                v_cartesian[i] = max_cartesian_acceleration * (t_f - cur_time)

    plt.figure(0)
    plt.ylabel('velocity, m')
    plt.xlabel('time, s')
    plt.title('cartesian velocity')
    x1 = time[0:-1]
    x2 = time[1:]
    y1 = v_cartesian[0:-1]
    y2 = v_cartesian[1:]
    plt.plot(x1, y1, x2, y2)
    plt.show()

    # done for cartesian velocity
    # now calculate joint velocities

    v = np.zeros(shape=(3, time.shape[0]))  # 3 because there are 3 joints


    pos = np.zeros(shape=(3, time.shape[0]))
    acc = np.zeros(shape=(3, time.shape[0]))

    pos[:, 0] = q0

    for (i,), cur_time in np.ndenumerate(time):
        if i == 0:
            continue
        v[:, i] = np.dot(jacobian_inverse(pos[:, i - 1]), get_joint_components(v_cartesian[i]))
        acc[:, i - 1] = (v[:, i] - v[:, i - 1]) * max_freq
        pos[:, i] = pos[:, i - 1] + (v[:, i] + v[:, i - 1]) / 2 * 0.01  # arithmetic mean

    return pos, v, acc



def arc_trajectory(x0, xf):
    dist = np.linalg.norm(xf - x0)
    dist_z = xf[2] - x0[2]
    dist_y = xf[1] - x0[1]
    dist_x = xf[0] - x0[0]
    dist_xy = np.sqrt(dist_x**2 + dist_y**2)
    sin_a = dist_z / dist
    cos_a = dist_xy / dist
    cos_b = dist_x / dist_xy
    sin_b = dist_y / dist_xy

    q0 = inv_kin(x0, L)
    qf = inv_kin(xf, L)

    def get_joint_components(module):
        return np.array([module * cos_a * cos_b, module * cos_a * sin_b, module * sin_a])

    t_ba = np.around(dist / max_cartesian_velocity, 2)
    t_b = np.around(max_cartesian_velocity / max_cartesian_acceleration, 2)

    time = 0
    v_cartesian = 0

    if t_b < t_ba:
        t_a = t_ba - t_b
        t_f = t_ba + t_b
        time = np.arange(0, t_f + 0.005, 0.01)  # add 0.005 to create array with correct size
        v_cartesian = np.zeros(time.shape[0])  # 3 because there are 3 joints

        for (i,), cur_time in np.ndenumerate(time):
            if cur_time < t_b:
                v_cartesian[i] = max_cartesian_acceleration * cur_time
            elif cur_time < t_a + t_b:
                v_cartesian[i] = max_cartesian_velocity
            else:
                v_cartesian[i] = max_cartesian_acceleration * (t_f - cur_time)

    else:
        t_b = np.around(np.sqrt(dist/max_joint_acceleration), 2)
        t_f = 2 * t_b
        time = np.arange(0, t_f + 0.005, 0.01)  # add 0.005 to create array with correct size
        v_cartesian = np.zeros(time.shape[0])  # 3 because there are 3 joints
        for (i,), cur_time in np.ndenumerate(time):
            if cur_time < t_b:
                v_cartesian[i] = max_cartesian_acceleration * cur_time
            else:
                v_cartesian[i] = max_cartesian_acceleration * (t_f - cur_time)

    plt.figure(0)
    plt.ylabel('velocity, m')
    plt.xlabel('time, s')
    plt.title('cartesian velocity')
    x1 = time[0:-1]
    x2 = time[1:]
    y1 = v_cartesian[0:-1]
    y2 = v_cartesian[1:]
    plt.plot(x1, y1, x2, y2)
    plt.show()

    # done for cartesian velocity
    # now calculate joint velocities

    v = np.zeros(shape=(3, time.shape[0]))  # 3 because there are 3 joints


    pos = np.zeros(shape=(3, time.shape[0]))
    acc = np.zeros(shape=(3, time.shape[0]))

    pos[:, 0] = q0

    for (i,), cur_time in np.ndenumerate(time):
        if i == 0:
            continue
        v[:, i] = np.dot(jacobian_inverse(pos[:, i - 1]), get_joint_components(v_cartesian[i]))
        acc[:, i - 1] = (v[:, i] - v[:, i - 1]) * max_freq
        pos[:, i] = pos[:, i - 1] + (v[:, i] + v[:, i - 1]) / 2 * 0.01  # arithmetic mean

    return pos, v, acc


# accepts 2 trajectories and returns combined trajectory with junction
def junction(t1, t2,):
    v1 = t1[1]
    new_pos = t1[0]
    new_acc = t1[2]
    v2 = t2[1]
    shift = junction_steps
    for i in range(-shift, 0):
        v1[:, i] += v2[:, shift+i]
        new_acc[:, i] = (v1[:, i] - v1[:, i - 1]) * max_freq
        new_pos[:, i] = new_pos[:, i - 1] + (v1[:, i] + v1[:, i - 1]) / 2 * 0.01
    v1 = np.hstack((v1, v2[:, shift:]))
    new_pos = np.hstack((new_pos, t2[0][:, shift:]))
    new_acc = np.hstack((new_acc, t2[2][:, shift:]))
    return new_pos, v1, new_acc


# J = jacobian()
# print(J)

def plan():
    # q0 = np.array([0, 1.046, 1.203])
    x1 = np.array([100.0, 0.0, 50.0])
    x2 = np.array([100.1, 0.0, 200.0])
    # q2 = inv_kin(x1, L)
    q3 = np.array([0, 1.046, 1.203])
    q4 = np.array([0.732, 1.046, 0.261])

    # trajectory1 = ptp_trajectory(q0, q1)
    trajectory0 = lin_trajectory(x1, x2)
    trajectory1 = lin_trajectory(x2, x1)
    trajectory2 = ptp_trajectory(trajectory1[0][:, -1], q3)
    trajectory3 = ptp_trajectory(q3, q4) # junction(trajectory0, trajectory1)
    # trajectory3 = ptp_trajectory(trajectory2[0][:, -1], q3)
    # trajectory4 = ptp_trajectory(q3, q4)

    t12_with_junction = junction(trajectory1, trajectory2)
    t13_with_junction = junction(t12_with_junction, trajectory3)

    # pos = t14_with_junction[0]
    # v = t14_with_junction[1]
    # acc = t14_with_junction[2]

    pos = t13_with_junction[0]
    v = t13_with_junction[1]
    acc = t13_with_junction[2]

    # motion_plot(pos, v, acc)

    return pos



