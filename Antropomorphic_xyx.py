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
                 lengths=None, save=False, offsets=None, directions=None):
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
            self.qs_offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
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
            self._Ts = st("RzTzRyTzRyTzRzTzRyTzRzTz",
                          [ 'q_0','l_0', 'q_1', 'l_1', 'q_2', 'l_2',
                           'q_3', 'l_3', 'q_4', 'l_4', 'q_5', 'l_5'])

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
    
    def inverse_kinematics(self, T, m=1, k=1, w=0):
        """
        Calculates inverse kinematics joint values qs from pose T
        Args:
            T (4x4 array like): Homogeneous pose matrix
            m (int, optional): Elbow up flag. Should be -1 or 1
            k (int, optional): Square root sign flag. Should be -1 or 1
            w (int, optional): Wrist rotation flag. Should be -1 or 1
        Returns:
            np.ndarray: Joint values, corresponding to T
                or zeros in case of failure
        """
        if abs(m) != 1:
            print("[WARNING] m can only be -1 or 1. Defaulting to 1")
            m = 1

        if abs(k) != 1:
            print("[WARNING] k can only be -1 or 1. Defaulting to 1")
            k = 1

        if abs(w) != 1 and w != 0:
            print("[WARNING] w can only be -1, 0 or 1. Defaulting to 1")
            w = 1

        T = sp.Matrix(T)
        Tz_inv = Transformation.get_Tz_inv(self._ls[5] + self._ls[4])
        #Tx_inv = Transformation.get_Tx_inv(self._ls[6])
        T_0 = T * Tz_inv

        x, y, z = float(T_0[0, 3]), float(T_0[1, 3]), float(T_0[2, 3])
        r = np.sqrt(x**2 + y**2)
        s = z - self._ls[0]
        l2 = self._ls[2] + self._ls[3]
        v = np.sqrt(r**2 + s**2)
        alpha_cos = (self._ls[1]**2 + l2**2 - v**2) / (2 * self._ls[1] * l2)



        # Check if the given position is reachable
        # if abs(q2_cos) > 1:
        #     print("[INFO] The configuration is not reachable")
        #     return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        q_0 = np.arctan2(y, x)
        alpha = np.arccos(alpha_cos)
        q_2 = np.pi - alpha
        
        beta = np.arcsin((l2 * np.sin(alpha)) / v)
        gamma = np.arctan2(s, r)

        
        q_1 = np.pi/2 - gamma - beta

        print('q_0 ==               ' ,q_0)
        print('q_1 ==               ' ,q_1)
        print('q_2 ==               ' ,q_2)

        value_pairs = self._value_pairs.copy()
        value_pairs.append(("q_0", q_0))
        value_pairs.append(("q_1", q_1))
        value_pairs.append(("q_2", q_2))

        T_012 = st("RzTzRyTzRyTzTz",
                          [ 'q_0','l_0', 'q_1', 'l_1', 'q_2', 'l_2','l_3'])
        self._T_012_inv = T_012.inv()

        T_012_inv = self._T_012_inv.evaluate_tuples(value_pairs)
        #Ryi_dq = Transformation.get_Ry(-self.dq)
        T_rot = T_012_inv * T_0

        r_11 = float(T_rot[0, 0])
        r_21 = float(T_rot[1, 0])
        r_31 = float(T_rot[2, 0])
        r_12 = float(T_rot[0, 1])
        r_13 = float(T_rot[0, 2])
        r_32 = float(T_rot[2, 1])
        r_23 = float(T_rot[1, 2])
        r_33 = float(T_rot[2, 2])        

        # if abs(r_11 - 1.0) > self.epsilon:
        #     q_3 = np.arctan2(r_21, -r_31)
        #     q_5 = np.arctan2(r_12, r_13)

        #     q_3 -= w * np.pi
        #     q_5 -= w * np.pi

        #     if abs(r_13) > self.epsilon:
        #         q_4 = np.arctan2(r_13 / np.cos(q_5), r_11)
        #     else:
        #         q_4 = np.arctan2(r_12 / np.sin(q_5), r_11)
        # else:
        #     print("[INFO] Orientation singularity case")
        #     q_3 = 0.0
        #     q_4 = 0.0
        #     q_5 = 0.0
        q_5 = np.arctan2(r_32, -r_31)
        q_3 = np.arctan2(r_23, r_13)
        q_4 = np.arctan2(np.sqrt(r_31**2 + r_32**2), r_33)


        qs = np.array([q_0, q_1, q_2, q_3, q_4, q_5])
        qs = np.multiply(qs - self.qs_offsets, self.qs_directions)

        out_of_limits = False
        for i in range(len(qs)):
            if qs[i] < self.qs_lim_rad[i][0] or qs[i] > self.qs_lim_rad[i][1]:
                print(f"[INFO] q_{i} = {np.rad2deg(qs[i]):.3f} "
                      f"(degrees) is out of limits")
                out_of_limits = True

        if out_of_limits:
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        return qs