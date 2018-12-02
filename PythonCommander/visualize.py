import matplotlib.pyplot as plt
import numpy as np

from forward_inverse_kinematics import forw_kin
from robot_configuration import L

def motion_plot(pos, v, acc):
        time = np.arange(0, v.shape[1] * 0.01 - 0.005, 0.01)

        plt.figure(1)
        plt.ylabel('position, rad')
        plt.xlabel('time, s')
        plt.title('joint positions')

        for i in range(0, v.shape[0]):
            plt.plot(time, pos[i, :], 'C'+str(i+1))
        plt.savefig('plots/joint_positions.png')
        plt.show()

        plt.figure(2)
        plt.ylabel('velocity, rad')
        plt.xlabel('time, s')
        plt.title('joint velocities')

        for i in range(0, v.shape[0]):
            plt.plot(time, v[i, :], 'C'+str(i+1))
        plt.savefig('plots/joint velocities.png')
        plt.show()

        plt.figure(3)
        plt.ylabel('rad/s^2')
        plt.xlabel('time, ms')
        plt.title('joint accelerations')
        cartesian_pos = np.zeros(shape=(3, pos.shape[1]))
        for i in range(0, v.shape[0]):
            plt.plot(time, acc[i, :], 'C'+str(i+1))
        plt.savefig('plots/joint_accelerations.png')
        plt.show()

        for i in range(0, pos.shape[1]):
            cartesian_pos[:, i] = forw_kin(pos[:, i].flatten(), L)

        # plt.figure(4)
        # ax = plt.axes(projection='3d')
        # ax.set_xlabel('X axis')
        # ax.set_ylabel('Y axis')
        # ax.set_zlabel('Z axis')
        # ax.plot3D(cartesian_pos[0, :].flatten(), cartesian_pos[1, :].flatten(), cartesian_pos[2:, ].flatten(), 'gray')
        # plt.show()

        plt.figure(4)
        plt.ylabel('position, m')
        plt.xlabel('time, s')
        plt.title('X cartesian axis')
        plt.plot(time, cartesian_pos[0])
        # plt.ylim([-3, 10])

        plt.savefig('plots/x_position.png')
        plt.show()


        plt.figure(5)
        plt.ylabel('position, m')
        plt.xlabel('time, s')
        plt.title('Y cartesian axis')
        plt.plot(time, cartesian_pos[1])
        # plt.ylim([-3, 7])
        plt.savefig('plots/y_position.png')
        plt.show()


        plt.figure(6)
        plt.ylabel('position, m')
        plt.xlabel('time, s')
        plt.title('Z cartesian axis')
        plt.plot(time, cartesian_pos[2])
        # plt.ylim([-7, 2])
        plt.savefig('plots/z_position.png')
        plt.show()
