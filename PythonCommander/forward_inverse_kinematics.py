import numpy as np
from robot_configuration import L


# function that takes coordinates in cartesian space and returns joint angles (IK)
def inv_kin(pos, L):
    x = pos[0]
    y = pos[1]
    z = pos[2]
    q1 = np.arctan2(y, x)
    x_new = np.sqrt(x ** 2 + y ** 2)
    y_new = z - L[0]
    s = np.sqrt(x_new ** 2 + y_new ** 2)
    q2_part1 = np.arctan2(y_new, x_new)
    q2_part2 = np.arccos((L[1]**2 + s**2 - L[2]**2) / (2 * L[1] * s))
    q2 = q2_part1 + q2_part2
    q3 = np.arctan2(L[1] * np.sin(q2) - y_new, x_new - L[1] * np.cos(q2))
    return [q1, q2, q3]


# function that takes joint angles and returns x,y,z
def forw_kin(q, L):
    x = L[1] * np.cos(q[1]) * np.cos(q[0]) + \
        L[2] * np.cos(q[2]) * np.cos(q[0])

    y = L[1] * np.cos(q[1]) * np.sin(q[0]) + \
        L[2] * np.cos(q[2]) * np.sin(q[0])

    z = L[0] + L[1] * np.sin(q[1]) - L[2] * np.sin(q[2])

    return [x, y, z]



def test():
    ############Test it!##################
    # set of desired (x,y) hand positions
    x = np.arange(150, 170, 1)
    y = np.arange(150, 170, 1)
    z = 130

    # threshold for printing out information, to find trouble spots
    thresh = .025

    count = 0
    total_error = 0
    # test it across the range of specified x and y values
    for xi in range(len(x)):
        for yi in range(len(y)):
            # run the inv_kin function, get the optimal joint angles
            xyz = [x[xi], y[yi], z]
            q = inv_kin(np.array([x[xi], y[yi], z]), L)
            # find the (x,y) position of the hand given these angles
            actual_xyz = forw_kin(q, L)
            # calculate the root squared error
            error = np.sqrt((np.array(xyz) - np.array(actual_xyz)) ** 2)
            # total the error
            total_error += error

            # if the error was high, print out more information
            if np.sum(error) > thresh:
                print('Final joint angles: ', q)
                print('Desired hand position: ', xyz)
                print('Actual hand position: ', actual_xyz)
                print('Error: ', error)
                print('-------------------------')

            count += 1

    print('\n---------Results---------')
    print('Total number of trials: ', count)
    print('Total error: ', total_error)
    print('-------------------------')
