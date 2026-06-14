#
# Copyright (c) 2023, Polytechnique Lab for Assistive and Rehabilitation Technologies.
# POLAR, Polytechnique Montreal
# All rights reserved.
#
# All arm parameters. ToDo: to be replaced by a URDF or an xml file.
# Author: Abolfazl Mohebbi <abolfazl.mohebbi@polymtl.ca> <polar@polymtl.ca>

import numpy as np


class xArmParams:
    def __init__(self):
        # self.arm = arm
        self.model_number = 4

        self.nActuatorCount = 7
        self.TimeStep = 0.004

        self.ToolMass = 0.0
        self.ToolCOM = np.array([0, 0, 0])

        self.LinksMass = self.getLinksMass()
        self.LinksCOM = self.getLinksCOM()

        self.GravityVector = np.array([0.0, 0.0, -9.81])

        self.DH_params = self.getDHparams()

        self.JointTorqueNoiseThreshold = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        self.CartesianWrenchNoiseThreshold = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0])


    def getLinksMass(self):
        """
        Returns the mass of the xArm7 links based on the specific hardware model.
        :param model_number: Integer (1, 2, 3, or 4)
        :return: numpy array of masses [kg]
        """
        model_number = self.model_number
        link_mass = np.zeros(8)

        if model_number == 1:
            # Model 1 (v1.2 Hardware)
            link_mass[0] = 2.9710  # base mass
            link_mass[1] = 2.117
            link_mass[2] = 1.716
            link_mass[3] = 1.485
            link_mass[4] = 1.574
            link_mass[5] = 1.209
            link_mass[6] = 1.214
            link_mass[7] = 0.17

        elif model_number == 2:
            # Model 2 (v1.3 Hardware - Your Robot XS13...)
            link_mass[0] = 2.9710  # base mass
            link_mass[1] = 2.46
            link_mass[2] = 1.916
            link_mass[3] = 1.69
            link_mass[4] = 1.774
            link_mass[5] = 1.357
            link_mass[6] = 1.362
            link_mass[7] = 0.17

        elif model_number == 3:
            # Model 3 (Transitional 2020 batches)
            link_mass[0] = 2.9710  # base mass
            link_mass[1] = 2.382
            link_mass[2] = 1.869
            link_mass[3] = 1.638
            link_mass[4] = 1.727
            link_mass[5] = 1.32
            link_mass[6] = 1.325
            link_mass[7] = 0.17

        elif model_number == 4:
            # Model 4 (v1.5+ Hardware)
            link_mass[0] = 2.9710  # base mass
            link_mass[1] = 2.53
            link_mass[2] = 2.166
            link_mass[3] = 1.98
            link_mass[4] = 2.08
            link_mass[5] = 1.36
            link_mass[6] = 1.345
            link_mass[7] = 0.173

        else:
            # Fallback/Error handling
            raise ValueError(f"Unknown xArm7 model number: {model_number}")

        return link_mass

    def getLinksCOM(self):
        """
        Returns the Center of Mass (COM) coordinates for xArm7 links.
        :param model_number: Integer (2, 3, or 4)
        :return: numpy array of shape (8, 3) in meters
        """
        model_number = self.model_number
        # Initialize array for Base + 7 Joints
        LinkCOMxyz = np.zeros([self.nActuatorCount + 1, 3])

        if model_number == 1:
            # xArm7 Model 1
            LinkCOMxyz[0][0] = 0.000  # base COM w.r.t. base frame
            LinkCOMxyz[0][1] = 0.000
            LinkCOMxyz[0][2] = 0.050

            LinkCOMxyz[1][0] = 0.00015
            LinkCOMxyz[1][1] = 0.02724
            LinkCOMxyz[1][2] = -0.01375

            LinkCOMxyz[2][0] = 0.00022
            LinkCOMxyz[2][1] = -0.1247
            LinkCOMxyz[2][2] = 0.0189

            LinkCOMxyz[3][0] = 0.0460
            LinkCOMxyz[3][1] = -0.0223
            LinkCOMxyz[3][2] = -0.00847

            LinkCOMxyz[4][0] = -0.06975
            LinkCOMxyz[4][1] = -0.1125
            LinkCOMxyz[4][2] = 0.0132

            LinkCOMxyz[5][0] = -0.00035
            LinkCOMxyz[5][1] = 0.0176
            LinkCOMxyz[5][2] = -0.0284

            LinkCOMxyz[6][0] = 0.06365
            LinkCOMxyz[6][1] = 0.03084
            LinkCOMxyz[6][2] = 0.0217

            LinkCOMxyz[7][0] = 0.000000
            LinkCOMxyz[7][1] = -0.00677
            LinkCOMxyz[7][2] = -0.01098

        if model_number == 2:
            # xArm7 Model 2 (v1.3 Hardware)

            LinkCOMxyz[0][0] = 0.000  # base COM w.r.t. base frame
            LinkCOMxyz[0][1] = 0.000
            LinkCOMxyz[0][2] = 0.050

            LinkCOMxyz[1][0] = 0.00013
            LinkCOMxyz[1][1] = 0.03010
            LinkCOMxyz[1][2] = -0.0120

            LinkCOMxyz[2][0] = 0.0002
            LinkCOMxyz[2][1] = -0.1296
            LinkCOMxyz[2][2] = 0.0169

            LinkCOMxyz[3][0] = 0.04676
            LinkCOMxyz[3][1] = -0.0253
            LinkCOMxyz[3][2] = -0.00746

            LinkCOMxyz[4][0] = 0.07066
            LinkCOMxyz[4][1] = -0.1166
            LinkCOMxyz[4][2] = 0.0117

            LinkCOMxyz[5][0] = -0.0003
            LinkCOMxyz[5][1] = 0.0156
            LinkCOMxyz[5][2] = -0.0253

            LinkCOMxyz[6][0] = 0.0650
            LinkCOMxyz[6][1] = 0.0334
            LinkCOMxyz[6][2] = 0.0213

            LinkCOMxyz[7][0] = 0.000000
            LinkCOMxyz[7][1] = -0.00677
            LinkCOMxyz[7][2] = -0.01098

        elif model_number == 3:
            # xArm7 Model 3 (Transitional 2020 batches)
            LinkCOMxyz[0][0] = 0.000
            LinkCOMxyz[0][1] = 0.000
            LinkCOMxyz[0][2] = 0.050

            # Link 1
            LinkCOMxyz[1][0] = 0.00013
            LinkCOMxyz[1][1] = 0.0294
            LinkCOMxyz[1][2] = -0.0124

            # Link 2
            LinkCOMxyz[2][0] = 0.0002
            LinkCOMxyz[2][1] = -0.12856
            LinkCOMxyz[2][2] = 0.01735

            # Link 3
            LinkCOMxyz[3][0] = 0.0466
            LinkCOMxyz[3][1] = -0.02463
            LinkCOMxyz[3][2] = -0.00768

            # Link 4
            LinkCOMxyz[4][0] = 0.0705
            LinkCOMxyz[4][1] = -0.11575
            LinkCOMxyz[4][2] = 0.012

            # Link 5
            LinkCOMxyz[5][0] = -0.00032
            LinkCOMxyz[5][1] = 0.01604
            LinkCOMxyz[5][2] = -0.026

            # Link 6
            LinkCOMxyz[6][0] = 0.0647
            LinkCOMxyz[6][1] = 0.0328
            LinkCOMxyz[6][2] = 0.0214

            # Link 7
            LinkCOMxyz[7][0] = 0.00000
            LinkCOMxyz[7][1] = -0.00677
            LinkCOMxyz[7][2] = -0.01098


        elif model_number == 4:
            # xArm7 Model 4 (v1.5+ Hardware)

            # Base Frame (Assuming standard base if not provided)
            LinkCOMxyz[0][0] = 0.000
            LinkCOMxyz[0][1] = 0.000
            LinkCOMxyz[0][2] = 0.050

            # Link 1
            LinkCOMxyz[1][0] = -0.000164
            LinkCOMxyz[1][1] = 0.03594
            LinkCOMxyz[1][2] = -0.0065

            # Link 2
            LinkCOMxyz[2][0] = -0.00021
            LinkCOMxyz[2][1] = -0.1389
            LinkCOMxyz[2][2] = 0.01683

            # Link 3
            LinkCOMxyz[3][0] = 0.0467
            LinkCOMxyz[3][1] = -0.0226
            LinkCOMxyz[3][2] = -0.00764

            # Link 4
            LinkCOMxyz[4][0] = 0.0707
            LinkCOMxyz[4][1] = -0.12454
            LinkCOMxyz[4][2] = 0.01227

            # Link 5
            LinkCOMxyz[5][0] = 0.00019
            LinkCOMxyz[5][1] = 0.0146
            LinkCOMxyz[5][2] = -0.02198

            # Link 6
            LinkCOMxyz[6][0] = 0.06824
            LinkCOMxyz[6][1] = 0.03342
            LinkCOMxyz[6][2] = 0.00264

            # Link 7
            LinkCOMxyz[7][0] = 0.0008
            LinkCOMxyz[7][1] = -0.00359
            LinkCOMxyz[7][2] = -0.01326

        else:
            raise ValueError(f"Unknown xArm7 model number: {model_number}")

        return LinkCOMxyz

    def getDHparams(self):
        d = np.array([267, 0, 293, 0, 342.5, 0, 97]) * 0.001
        alpha = np.array([-np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, 0])  # in radians
        a = np.array([0, 0, 52.5, 77.5, 0, 76, 0]) * 0.001
        offset_rad = np.array([0, 0, 0, 0, 0, 0, 0])  # in radians
        DH_params = {
            "d": d,
            "alpha": alpha,
            "a": a,
            "offset_rad": offset_rad
        }
        return DH_params

    def getModifiedDHparams(self):  # NOT USED YET
        d = np.array([267, 0, 293, 0, 342.5, 0, 97]) * 0.001
        alpha = np.array([0, -np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2])  # in radians
        a = np.array([0, 0, 0, 52.5, 77.5, 0, 76]) * 0.001
        offset_rad = np.array([0, 0, 0, 0, 0, 0, 0])  # in radians
        DH_params = {
            "d": d,
            "alpha": alpha,
            "a": a,
            "offset_rad": offset_rad
        }
        return DH_params

    def getModifiedCartesianAdmittanceParams(self):
        # set tool impedance parameters:
        K_pos = 300  # x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
        K_ori = 4  # Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)

        # Attention: for M and J, smaller value means less effort to drive the arm,
        # but may also be less stable, please be careful.
        M_i = float(0.6)  # x/y/z equivalent mass; range: 0.02 ~ 1 kg
        J_i = M_i * 0.01  # Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
        M_x = np.diag([M_i, M_i, M_i, J_i, J_i, J_i])  # 6*6 Matrix
        K_x = np.diag([K_pos, K_pos, K_pos, K_ori, K_ori, K_ori])  # 6*6 Matrix
        B_x = np.diag([0, 0, 0, 0, 0, 0])  # B(damping) is reserved, give zeros  # 6*6 Matrix
        return M_x, B_x, K_x


    def getModifiedJointAdmittanceParams(self):
        # set tool impedance parameters:
        K_pos = 300  # x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
        K_ori = 4  # Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)

        # Attention: for M and J, smaller value means less effort to drive the arm,
        # but may also be less stable, please be careful.
        M_i = float(0.06)  # x/y/z equivalent mass; range: 0.02 ~ 1 kg
        J_i = M_i * 0.01  # Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
        M = np.diag([M_i, M_i, M_i, M_i, J_i, J_i, J_i])  # 6*6 Matrix
        K = np.diag([K_pos, K_pos, K_pos, K_pos, K_ori, K_ori, K_ori])  # 6*6 Matrix
        B = np.diag([0, 0, 0, 0, 0, 0, 0])  # B(damping) is reserved, give zeros  # 6*6 Matrix
        return M, B, K


    def getJointAdmittanceParams(self):

        B_arr = np.array([28.64, 28.64, 28.64, 28.64, 19.09, 19.09, 19.09]) * 0.5
        settlingTime = 1.0
        M_arr = np.ones(self.nActuatorCount)
        M_arr = (settlingTime / 3.0) * B_arr
        M = np.diag(M_arr)
        K = np.zeros([self.nActuatorCount, self.nActuatorCount])
        B = np.diag(B_arr)  # 6*6 Matrix
        return M, B, K


    def getCartesianAdmittanceParams(self):
        B_x = np.diag([200.0, 200.0, 200.0, 500, 500, 500]) * 0.4 * 0.001  # 0.001 because of meter to millimeters
        settlingTime = 1.0
        M_x = (settlingTime / 3.0) * B_x
        K_x = np.diag([0, 0, 0, 0, 0, 0])
        return M_x, B_x, K_x


    def getNullSpaceAdmittanceParams(self):
        B_arr = np.array([28.64, 28.64, 28.64, 28.64, 19.09, 19.09, 19.09]) * 0.5
        settlingTime = 1.0
        M_arr = np.ones(self.nActuatorCount)
        M_arr = (settlingTime / 3.0) * B_arr
        M_N = np.diag(M_arr)
        K_N = np.zeros([self.nActuatorCount, self.nActuatorCount])
        B_N = np.diag(B_arr)  # 6*6 Matrix
        return M_N, B_N, K_N






