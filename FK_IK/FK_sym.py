"""
Symbolic transformation usage example
"""
from utils.robo_math import SymbolicTransformation as st


def main():
    T = st("RzRyRz",[ 'q_3','q4', 'q5'])

    print('T_w = ')
    T.print()

if __name__ == '__main__':
    main()