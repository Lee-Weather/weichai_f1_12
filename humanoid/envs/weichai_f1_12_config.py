# SPDX-FileCopyrightText: Copyright (c) 2024 Beijing RobotEra TECHNOLOGY CO.,LTD.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Weichai F1 12-DOF lower-body robot.

The observation, action, and reward layout follows the X1 12-DOF walking task.
Robot assets are kept separate so the original X1 task remains available.
"""

from humanoid.envs.x1.x1_dh_stand_config import X1DHStandCfg, X1DHStandCfgPPO


class WeichaiF112Cfg(X1DHStandCfg):
    # F1 joint frames and zero pose differ from the X1 model. Keep its neutral
    # standing pose independent of the X1's crouched nominal angles.
    class init_state(X1DHStandCfg.init_state):
        default_joint_angles = {
            'left_hip_pitch_joint': 0.0,
            'left_hip_roll_joint': 0.0,
            'left_hip_yaw_joint': 0.0,
            'left_knee_pitch_joint': 0.0,
            'left_ankle_pitch_joint': 0.0,
            'left_ankle_roll_joint': 0.0,
            'right_hip_pitch_joint': 0.0,
            'right_hip_roll_joint': 0.0,
            'right_hip_yaw_joint': 0.0,
            'right_knee_pitch_joint': 0.0,
            'right_ankle_pitch_joint': 0.0,
            'right_ankle_roll_joint': 0.0,
        }

    class rewards(X1DHStandCfg.rewards):
        # F1-specific inverse-kinematics offsets from the zero-angle stance.
        # At swing peak the foot lifts 4 cm with its horizontal position and
        # ankle orientation held fixed. Left/right offsets are mirrored.
        final_swing_joint_delta_pos = [
            -0.450773, 0.0, 0.0, 0.767611, -0.316807, 0.0,
            -0.450773, 0.0, 0.0, 0.767611, -0.316807, 0.0,
        ]

    class asset(X1DHStandCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/weichai_f1_12/urdf/weichai_f1_12.urdf'
        xml_file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/weichai_f1_12/mjcf/weichai_f1_12.xml'
        name = "weichai_f1_12"
        foot_name = "ankle_roll"
        knee_name = "knee_pitch"


class WeichaiF112CfgPPO(X1DHStandCfgPPO):
    class runner(X1DHStandCfgPPO.runner):
        experiment_name = 'weichai_f1_12_dh_stand'
