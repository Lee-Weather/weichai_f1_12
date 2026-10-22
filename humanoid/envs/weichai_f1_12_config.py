# SPDX-FileCopyrightText: Copyright (c) 2024 Beijing RobotEra TECHNOLOGY CO.,LTD.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Weichai F1 12-DOF lower-body robot.

Standing angles, reference gait, control, rewards, and PPO parameters inherit
the X1 baseline. F1 assets plus the two hip-pitch sign corrections required by
F1's non-mirrored leg axes are overridden here.
"""

from humanoid.envs.x1.x1_dh_stand_config import X1DHStandCfg, X1DHStandCfgPPO


class WeichaiF112Cfg(X1DHStandCfg):
    # Keep the complete X1 training configuration as the comparison baseline.
    class asset(X1DHStandCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/weichai_f1_12/urdf/weichai_f1_12.urdf'
        xml_file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/weichai_f1_12/mjcf/weichai_f1_12.xml'
        name = "weichai_f1_12"
        foot_name = "ankle_roll"
        knee_name = "knee_pitch"

    class init_state(X1DHStandCfg.init_state):
        # F1 hip pitch axes are (0,1,0) on both legs (no mirror), so mirrored
        # signs produce a split stance: right foot 0.41 m behind and 0.12 m
        # below the left. Hip roll/yaw keep X1's opposite signs, which is correct.
        default_joint_angles = {
            **X1DHStandCfg.init_state.default_joint_angles,
            'right_hip_pitch_joint': 0.4,
        }

    class rewards(X1DHStandCfg.rewards):
        # Same rule for the swing reference: index 6 is right hip pitch.
        final_swing_joint_delta_pos = list(
            X1DHStandCfg.rewards.final_swing_joint_delta_pos)
        final_swing_joint_delta_pos[6] = 0.25


class WeichaiF112CfgPPO(X1DHStandCfgPPO):
    class runner(X1DHStandCfgPPO.runner):
        experiment_name = 'weichai_f1_12_dh_stand'
