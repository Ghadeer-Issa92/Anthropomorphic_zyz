import pickle
from pathlib import Path

import numpy as np
import sympy as sp

from robot import Robot
from utils.robo_math import SymbolicTransformation as st
from utils.robo_math import Transformation
from utils.plot_utils import TransformationPlotter


class Antropomorphic_xyx(Robot):
    qs_lim_deg = ((-185.0, 185.0),
                  (-90.0, 90.0),
                  (-156.0, 156.0),
                  (-360.0, 360.0),
                  (-125.0, 125.0),
                  (-360.0, 360.0))
    
    def __init__(self, T_base=None, T_tool=None,
                 lengths=None, save=True, offsets=None, directions=None):
        """
        Prepares all necessary values and loads pickled matrices
        Args:
            T_base (None, optional): Transformation from the world frame
                to the base frame
            T_tool (None, optional): Transformation from the end-effector
                frame to the tool frame
        """
        self.set_transforms(T_base, T_tool)
        self.set_lengths(lengths)
        self.set_joint_offsets(offsets)
        self.set_joint_directions(directions)

        self._generate_value_pairs()
        self._calculate_limits_radians()
        self._save = save
        if self._save:
            self.fk_data_path = Path(
                "data/antro_forward_kinematics.pkl")
            self.ik_data_path = Path(
                "data/antro_inverse_kinematics.pkl")
        self._precalculate_data()
    
    def set_lengths(self, lengths):
        if lengths is None:
            self._ls = (100.0, 150.0, 200.0, 20.0, 20.0, 20.0)
        else:
            self._ls = lengths

    def set_joint_offsets(self, offsets):
        if offsets is None:
            angle = np.pi / 2
            self.qs_offsets = np.array([0.0, 0.0, angle, 0.0, 0.0, 0.0])
        else:
            self.qs_offsets = offsets
    
    def set_joint_directions(self, directions):
        if directions is None:
            self.qs_directions = np.array([1, 1, 1, 1, 1, 1])
        else:
            self.qs_directions = directions

    def _generate_value_pairs(self):
        """
        Generates name-value tuples for sympy substitution
        """
        value_pairs = []
        for i in range(len(self._ls)):
            value_pairs.append((f"l_{i}", self._ls[i]))

        #self.d = np.sqrt(self._ls[4] * self._ls[4] + self._ls[5] * self._ls[5])
        #self.dq = np.arctan2(self._ls[4], self._ls[5])

        #value_pairs.append(("d", self.d))
        #value_pairs.append(("dq", self.dq))

        self._value_pairs = value_pairs

    def _calculate_limits_radians(self):
        """
        Converts joint limits from degrees to radians
        """
        self.qs_lim_rad = tuple(
            (np.deg2rad(x[0]), np.deg2rad(x[1])) for x in self.qs_lim_deg)

    def _precalculate_data(self):
        """
        Precalculates and pickles constant matrices
        """
        if self._save and self.fk_data_path.is_file():
            with open(self.fk_data_path, 'rb') as input:
                self._Ts = pickle.load(input)
        else:
            # 
            self._Ts = st("RzTzRyTzRyTzRxTzRyTzRx",
                          [ 'q_0','l_0', 'q_1', 'l_1', 'q_2', 'l_2',
                           'q_3', 'l_3', 'q_4', 'l_4', 'q_5'])

            # self._Ts = st("TzRzTzTxRyTxRyRyiTxRyRxRyRxTx",
            #               ['l_0', "q_0", "l_1", "l_2",
            #                "q_1", "l_3", "q_2", "dq",
            #                "d", "dq", "q_3", "q_4", "q_5", "l_6"])
            self._Ts.substitute(self._value_pairs)
        #fix for IK 
        # if self._save and self.ik_data_path.is_file():
        #     with open(self.ik_data_path, 'rb') as input:
        #         self._T_012_inv = pickle.load(input)
        # else:
        #     # Calculate rotation angles
        #     T_012 = st("RzTzTxRyTxRyRyiTx",
        #                ["q_0", "l_1", "l_2", "q_1", "l_3", "q_2", "dq", "d"])
        #     self._T_012_inv = T_012.inv()

        # Save data
        if self._save and not self.fk_data_path.is_file():
            with open(self.fk_data_path, 'wb') as output:
                pickle.dump(self._Ts, output, pickle.HIGHEST_PROTOCOL)

        # if self._save and not self.ik_data_path.is_file():
        #     with open(self.ik_data_path, 'wb') as output:
        #         pickle.dump(self._T_012_inv, output, pickle.HIGHEST_PROTOCOL)

    def forward_kinematics(self, q_values, plot=True):
        """
        Calculates forward kinematics of the tool pose given values of joints
        Args:
            q_values (list of float): Values of joints
            plot (bool, optional): Flag to plot the result
        Returns:
            4x4 np.ndarray: Homogeneous tool pose
        """
        # Account for the opposite rotation directions and offsets
        q_values = np.multiply(q_values, self.qs_directions) + self.qs_offsets

        qs_dict = {}
        for i in range(len(q_values)):
            qs_dict[sp.symbols(f"q_{i}")] = q_values[i]

        self._numeric_frames = []
        for frame in self._Ts.frames:
            self._numeric_frames.append(frame.evalf(subs=qs_dict))

        T = self.T_base * self._numeric_frames[-1] * self.T_tool

        if plot:
            self._tp = TransformationPlotter()
            self._show_fk()

        return np.array(T, dtype=np.float)

    def _show_fk(self):
        """
        Plots current pose of the robot
        """
        frames = [self.T_base]

        for frame, var in zip(self._numeric_frames, self._Ts.variables):
            if var[0] == 'q':
                frames.append(self.T_base * frame)

        frames.append(self.T_base * self._numeric_frames[-1])
        frames.append(frames[-1] * self.T_tool)

        self._tp.plot_numeric_frames(frames)