"""
Fanuc FK and IK usage example
"""
import numpy as np

from Anthropomorphic_zyz import Anthropomorphic_zyz


def main():
    np.set_printoptions(suppress=True)

    T_base = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T_tool = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    robot = Anthropomorphic_zyz(T_base=T_base, T_tool=T_tool)
    pi = np.pi
    qs = [pi/4, pi/2, pi/4, 0.0, 0.0, 0.0]

    T = robot.forward_kinematics(qs, plot=True)

    print("Forward kinematics:")
    print(T)
    print()

    print("Real Qs:")
    print(qs)

    T_IK = T

    qs = robot.inverse_kinematics(T_IK)

    print()
    print("IK Qs:")
    print(qs)

    T_res = robot.forward_kinematics(qs, plot=True)

    print()
    print("After inverse kinematics:")
    print(T_res)

if __name__ == '__main__':
    main()