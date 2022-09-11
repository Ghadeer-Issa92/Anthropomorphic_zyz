"""
Fanuc FK and IK usage example
"""
import numpy as np

from Antropomorphic_xyx import Antropomorphic_xyx


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

    robot = Antropomorphic_xyx(T_base=T_base, T_tool=T_tool)

    qs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    T = robot.forward_kinematics(qs, plot=True)

    print("Forward kinematics:")
    print(T)
    print()

    print("Real Qs:")
    print(qs)

if __name__ == '__main__':
    main()