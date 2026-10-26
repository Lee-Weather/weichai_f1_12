# SPDX-FileCopyrightText: Copyright (c) 2024 Beijing RobotEra TECHNOLOGY CO.,LTD.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Weichai F1 12-DOF lower-body robot.

Standing angles, reference gait, control, rewards, and PPO parameters inherit
the X1 baseline. F1 assets plus the left hip-pitch correction required by F1's
non-mirrored leg axes are overridden here.
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
        # X1's hip pitch joints sit in rotated frames (axis (0,0,1), rpy -0.7854),
        # so its numeric angles do not describe the same pose on F1, whose hip
        # pitch axis is (0,1,0) on both legs. On the F1 URDF the X1 values put the
        # left foot 0.34 m behind and 0.12 m above the right one; both legs must
        # use the right leg's -0.4 to get feet under the base (stand height 0.581 m,
        # mirror residual 0.0000 m, consistent with X1's own 0.610 m at 0.61 target).
        # Hip roll: F1's hip-roll axis points the opposite way from X1's rotated one
        # (czy/analysis/exp_ada_4/joint_axis_diff.py), so X1's +0.05/-0.05 becomes
        # adduction here and narrows the stance instead of splaying it. The swing
        # delta [1]/[7] is flipped for the same reason.
        default_joint_angles = {
            **X1DHStandCfg.init_state.default_joint_angles,
            'left_hip_pitch_joint': -0.4,
            'left_hip_roll_joint': -0.05,
            'right_hip_roll_joint': 0.05,
        }

    class rewards(X1DHStandCfg.rewards):
        # Same rule for the swing reference: index 0 is left hip pitch, which must
        # match the right leg (index 6) at -0.25; indices 1/7 are the mirrored
        # hip-roll pair, so they must change sign together with the defaults.
        final_swing_joint_delta_pos = list(
            X1DHStandCfg.rewards.final_swing_joint_delta_pos)
        final_swing_joint_delta_pos[0] = -0.25
        final_swing_joint_delta_pos[1] = -0.05
        final_swing_joint_delta_pos[7] = 0.05
        # Calibrated on the F1 URDF at the default stance above (leg_metrics.py):
        # root-to-ankle 0.545 m + sole offset 0.034 m. X1's 0.61 target is above
        # F1's kinematic maximum (0.566 + 0.034 = 0.600 m fully straight).
        base_height_target = 0.579
        # Measured on the F1 foot collision mesh; X1's value overstates it by 7 mm.
        feet_to_ankle_distance = 0.034


class WeichaiF112CfgPPO(X1DHStandCfgPPO):
    class runner(X1DHStandCfgPPO.runner):
        experiment_name = 'weichai_f1_12_dh_stand'
