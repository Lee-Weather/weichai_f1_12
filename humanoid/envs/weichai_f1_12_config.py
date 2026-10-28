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
        #
        # exp_ada_11: tried the knee gain 100 -> 140 (damping 14) on this same
        # stance and the policy collapsed (reward 25.7 / episode 734 at iter 799
        # vs exp_ada_8's 65.6 / 1786, ref_joint_pos 0.49 -> 0.11). The knee is far
        # more gain-sensitive than the ankle: 1.4x already breaks standing.
        #
        # exp_ada_12: take the gentler half of that step instead, 100 -> 115 with
        # damping 11.5 (same ~100 ms ratio). At 0.474 rad of extension travel that
        # is 54 N*m versus the inherited 47, so it only trims the 2.484 rad
        # overshoot exp_ada_9 needed; it is a probe, not an expected fix.
        #
        # exp_ada_15: the hip has the opposite problem - plenty of travel (+-1.54)
        # but 3.18x X1's inertia about the hip axis (0.878 vs 0.276 kg*m^2, mostly
        # F1's heavier shank and foot). At X1's Kp 30 the closed-loop bandwidth is
        # only 5.85 rad/s and the replay showed large commands (|action| 0.67-0.87)
        # with almost no motion (std 0.07-0.09 rad). A closed-loop replay sweep on
        # the same checkpoint (czy/analysis/exp_ada_14/hip_kp_compare.py) gave:
        # Kp 30 -> 50 -> 95 resets 6/6/5, hip p95 error 1.62 -> 1.59 -> 1.23 rad,
        # 0.4 m/s stage vx -0.057 -> +0.028 -> +0.141, hip L/R correlation
        # 0.589 -> 0.450 -> 0.235. 95/9.5 restores X1's 10.42 rad/s and zeta 0.52
        # (95 * 1.54 = 146 N*m only saturates past 1.37 rad of error).
        stiffness = {**X1DHStandCfg.control.stiffness,
                     'ankle_pitch_joint': 75,
                     'knee_pitch_joint': 115,
                     'hip_pitch_joint': 95}
        damping = {**X1DHStandCfg.control.damping,
                   'ankle_pitch_joint': 1.25,
                   'knee_pitch_joint': 11.5,
                   'hip_pitch_joint': 9.5}

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
        # exp_ada_10 tried a 0.65 knee default (root-to-ankle 0.5256 m, target
        # 0.560) but the probe stalled, so exp_ada_11 reverts the whole stance to
        # this value and raises the knee gain instead.
        base_height_target = 0.579
        # Measured on the F1 foot collision mesh; X1's value overstates it by 7 mm.
        feet_to_ankle_distance = 0.034


class WeichaiF112CfgPPO(X1DHStandCfgPPO):
    class runner(X1DHStandCfgPPO.runner):
        experiment_name = 'weichai_f1_12_dh_stand'
