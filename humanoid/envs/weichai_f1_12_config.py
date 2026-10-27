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
            'left_hip_pitch_joint': -0.56,
            'left_hip_roll_joint': -0.05,
            'right_hip_roll_joint': 0.05,
            # exp_ada_10: F1's knee is unilateral ([0, 1.5708]) like the ankle, so
            # X1's 0.49 default leaves only 0.474 rad of extension and Kp 100 can
            # ask for 47 N*m while the replay demanded 110 N*m (right knee was
            # saturated 13.3 % of the time in exp_ada_9). Raising the default to
            # 0.65 gives 0.634 rad (+34 %, 63 N*m) without touching the gains.
            # The hip pitch moves with it (-0.56) so the foot pitch stays at the
            # baseline -0.12 rad; FK (czy/analysis/exp_ada_10/knee_sweep.py):
            # foot x +0.105 m, ankle z -0.5256 m, height 0.5596 m, mirror residual
            # 0.0000 m, swing clearance 0.0387 -> 0.0490 m.
            'left_knee_pitch_joint': 0.65,
            'right_knee_pitch_joint': 0.65,
        }

    class control(X1DHStandCfg.control):
        # PD gains are inherited from X1 except for the ankle pitch. X1's ankle has
        # +-3.14 rad of travel, so Kp 35 still yields 35*3.14 = 110 N*m. F1's ankle
        # stops at -0.5236 rad with the default already at -0.21, i.e. only 0.30 rad
        # of plantarflexion left, so the same gain tops out at 10.6 N*m while the
        # measured demand is 15-60 N*m. The exp_ada_6 policy paid for that by
        # commanding targets 1.4-1.9 rad outside the joint range (44-59 % of steps)
        # and carrying a constant 0.41 rad PD error. exp_ada_7 tried 150 and the
        # policy never learned to stand (reward 18 vs exp_ada_5's 55 at iter 900,
        # episode 463 vs 1075 steps, every posture/reference term collapsed while
        # the torque-limit penalty went DOWN), so the ankle was too stiff rather
        # than torque-starved. Halfway: 75 turns the 0.41 rad bias into
        # 0.41*35/75 = 0.19 rad and reaches 0.31*75 = 23 N*m at the travel edge
        # (2.2x the inherited 10.6 N*m), still far below 0.85*241.9 = 205 N*m.
        # Damping keeps the inherited ~17 ms damping/Kp ratio used by exp_ada_7.
        stiffness = {**X1DHStandCfg.control.stiffness,
                     'ankle_pitch_joint': 75}
        damping = {**X1DHStandCfg.control.damping,
                   'ankle_pitch_joint': 1.25}

    class rewards(X1DHStandCfg.rewards):
        # Same rule for the swing reference: index 0 is left hip pitch, which must
        # match the right leg (index 6) at -0.25; indices 1/7 are the mirrored
        # hip-roll pair, so they must change sign together with the defaults.
        final_swing_joint_delta_pos = list(
            X1DHStandCfg.rewards.final_swing_joint_delta_pos)
        final_swing_joint_delta_pos[0] = -0.25
        final_swing_joint_delta_pos[1] = -0.05
        final_swing_joint_delta_pos[7] = 0.05
        # X1's -0.16 puts the swing ankle at -0.37 rad, only 0.14 rad from F1's
        # -0.5236 stop, so the reference itself saturates on every swing. Halving
        # it keeps the sign of the X1 gait and doubles the remaining margin.
        final_swing_joint_delta_pos[4] = -0.08
        final_swing_joint_delta_pos[10] = -0.08
        # Calibrated on the F1 URDF at the default stance above (leg_metrics.py):
        # root-to-ankle 0.545 m + sole offset 0.034 m. X1's 0.61 target is above
        # F1's kinematic maximum (0.566 + 0.034 = 0.600 m fully straight).
        # exp_ada_10: the knee default 0.49 -> 0.65 shortens the leg, so the same
        # FK gives root-to-ankle 0.5256 m (+0.034 = 0.5596). Keep the target at
        # the achievable stance height instead of letting the reward fight the pose.
        base_height_target = 0.560
        # Measured on the F1 foot collision mesh; X1's value overstates it by 7 mm.
        feet_to_ankle_distance = 0.034


class WeichaiF112CfgPPO(X1DHStandCfgPPO):
    class runner(X1DHStandCfgPPO.runner):
        experiment_name = 'weichai_f1_12_dh_stand'
