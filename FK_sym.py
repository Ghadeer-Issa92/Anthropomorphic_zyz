"""
Symbolic transformation usage example
"""
from utils.robo_math import SymbolicTransformation as st


def main():
    T = st("RzTzRyTzRyTzRxTzRyTzRx",
                          [ 'q_0','l_0', 'q_1', 'l_1', 'q_2', 'l_2',
                           'q_3', 'l_3', 'q_4', 'l_4', 'q_5'])

    print('T = ')
    T.print()

if __name__ == '__main__':
    main()