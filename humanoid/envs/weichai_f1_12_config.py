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
        # exp_ada_15 tried the full inertia match (95/9.5) and FAILED from scratch:
        # iter 799 reward 4.17 / episode 245 versus exp_ada_8's 65.56 / 1786, with
        # reward stuck at 4-8 for the whole run. The replay gain at Kp 95 was a
        # policy trained at Kp 30 re-run with a stiffer hip, so it does not carry
        # over to learning from scratch. F1's hip is the worst case for that:
        # 0.878 kg*m^2 but only 130 N*m of effort, so a large gain saturates the
        # joint during the random-action phase and standing never forms.
        # exp_ada_16 takes the intermediate value the replay already showed stable
        # (Kp 50: resets 6, hip p95 error 1.59, 0.4 m/s stage vx +0.028) - 1.67x
        # X1 - which is the order of magnitude the from-scratch runs tolerate
        # (ankle 2.14x passed, knee 1.4x failed).
        # exp_ada_26: the last un-calibrated pair of joints is the LATERAL one.
        # exp_ada_23's replay shows the policy cannot move its COM over one foot:
        # single support 7.5 %, right hip roll carries a 0.48 rad PD error (left 0.27)
        # and peaks at the 110.5 N*m clip, right ankle roll sits on its 0.513 stop with
        # 11.7 % of raw targets out of range. exp_ada_25 then removed the hop income and
        # charged -0.5/s for flight, and the policy STILL flew 7.6 % of the time, so the
        # missing capability is weight transfer, not the incentive. Inertia-matched gains
        # are hip_roll 66 / 4.9 (1.65x) and ankle_roll 73 / 1.0 (2.09x); take 1.5x and
        # 1.43x instead, the corner the from-scratch runs already tolerate (hip_pitch 50
        # trains while 95 fails, ankle pitch 75 trains while 150 fails). Torque headroom
        # is large (hip_roll 19 N*m mean of 110.5, ankle_roll 6.8 of 205), so unlike the
        # knee this is not a saturation trade.
        stiffness = {**X1DHStandCfg.control.stiffness,
                     'ankle_pitch_joint': 75,
                     'knee_pitch_joint': 115,
                     'hip_pitch_joint': 50,
                     'hip_roll_joint': 60,
                     'ankle_roll_joint': 50}
        damping = {**X1DHStandCfg.control.damping,
                   'ankle_pitch_joint': 1.25,
                   'knee_pitch_joint': 11.5,
                   'hip_pitch_joint': 4.9,
                   'hip_roll_joint': 4.5,
                   'ankle_roll_joint': 0.75}

    class rewards(X1DHStandCfg.rewards):
        # Same rule for the swing reference: index 0 is left hip pitch, which must
        # match the right leg (index 6) at -0.25; indices 1/7 are the mirrored
        # hip-roll pair, so they must change sign together with the defaults.
        final_swing_joint_delta_pos = list(
            X1DHStandCfg.rewards.final_swing_joint_delta_pos)
        # exp_ada_23: hip swing 0.25 -> 0.35 on BOTH legs (axis (0,1,0), so the same
        # sign is correct). exp_ada_22 showed the knee cannot fix direction without
        # losing lift: knee 0.38 put the swing foot forward but cut real swing
        # phases 34 -> 12. The FK grid in czy/analysis/exp_ada_22/swing_grid.py
        # shows hip -0.35 with knee 0.50 gives dx +0.0238 m forward AND 0.0611 m
        # lift, so direction and clearance stop competing.
        final_swing_joint_delta_pos[0] = -0.35
        final_swing_joint_delta_pos[6] = -0.35
        final_swing_joint_delta_pos[1] = -0.05
        final_swing_joint_delta_pos[7] = 0.05
        # X1's -0.16 puts the swing ankle at -0.37 rad, only 0.14 rad from F1's
        # -0.5236 stop, so the reference itself saturates on every swing. Halving
        # it keeps the sign of the X1 gait and doubles the remaining margin.
        final_swing_joint_delta_pos[4] = -0.08
        final_swing_joint_delta_pos[10] = -0.08
        # exp_ada_17's replay settled the gait question: the policy already tracks the
        # reference (ref_joint_pos 0.74, targets oscillating at 1.40-1.50 Hz) but the
        # feet never leave the ground (sole height flat at 0.034 m, feet_air_time
        # 0.0061, double support 85-100 %). Two F1-specific reasons:
        #  1. X1's swing deltas put only 0.0326 m of horizontal travel and ~0.01 m of
        #     lift on F1's foot, i.e. the reference itself is a shuffle. Raising the
        #     swing knee 0.35 -> 0.50 and target_feet_height 0.03/0.06 -> 0.05/0.10
        #     asks for a real step instead.
        #  2. Hip bandwidth sits at the gait frequency: omega = sqrt(50/0.878) =
        #     7.55 rad/s (1.20 Hz) versus cycle_time 0.7 s = 8.98 rad/s (1.43 Hz), so
        #     the swing has to be completed in less time than the joint needs. At
        #     1.1 s the reference drops to 5.71 rad/s, giving the hip a 1.32x margin.
        cycle_time = 1.1
        target_feet_height = 0.05
        target_feet_height_max = 0.10
        # exp_ada_22: the swing knee goes 0.50 -> 0.38. exp_ada_21 finally learned an
        # alternating gait (34 swing phases >= 0.15 s, hip L/R correlation -0.19) but
        # walked BACKWARD at ~0.5 m/s under forward commands, and the reference was
        # the reason: with cycle_time 1.1 s and hip delta -0.25, FK in
        # czy/analysis/exp_ada_21/swing_sweep.py puts the swing foot 0.0384 m FORWARD
        # of the default at knee delta 0.30, 0.0218 at 0.35, 0.0119 at 0.38, 0.0053
        # at 0.40, and BEHIND it (-0.0270) at 0.50 - the sinusoid flexes the knee in
        # the same phase as the hip, so past ~0.41 the retraction wins and the
        # reference becomes a backward step. Lift and forward travel trade off
        # directly (3.0 cm at 0.30, 4.2 cm at 0.38, 5.2 cm at 0.50), so 0.38 keeps
        # 4.2 cm of clearance while restoring a forward stride.
        # exp_ada_22 tried knee 0.38 (forward stride restored) but the policy then
        # stopped stepping at all: real swing phases fell 34 -> 12 and the legs went
        # back in phase (hip L/R correlation -0.19 -> +0.82). exp_ada_23 keeps the
        # knee at 0.50 (0.0611 m lift with the larger hip swing) and buys the
        # forward direction from the hip instead.
        final_swing_joint_delta_pos[3] = 0.50
        final_swing_joint_delta_pos[9] = 0.50
        # Calibrated on the F1 URDF at the default stance above (leg_metrics.py):
        # root-to-ankle 0.545 m + sole offset 0.034 m. X1's 0.61 target is above
        # F1's kinematic maximum (0.566 + 0.034 = 0.600 m fully straight).
        # exp_ada_10 tried a 0.65 knee default (root-to-ankle 0.5256 m, target
        # 0.560) but the probe stalled, so exp_ada_11 reverts the whole stance to
        # this value and raises the knee gain instead.
        base_height_target = 0.579
        # Measured on the F1 foot collision mesh; X1's value overstates it by 7 mm.
        feet_to_ankle_distance = 0.034

        # exp_ada_21: exp_ada_20's replay showed the rebalanced rewards pay for
        # flailing rather than for moving. Touchdown counts rose 5-10x, but 45 of
        # 80 airborne runs lasted < 0.05 s and every commanded stage ended in a
        # backward fall (stage vx -0.112 / -0.141 / -0.362), because nothing in
        # the reward says the step has to produce displacement: feet_air_time pays
        # for lift, foot_slip was cut 3.3x and contact_forces 2.5x, while the
        # velocity terms stayed at X1's weight. exp_ada_20's joint audit also
        # closed the gain levers: hip_pitch averages 6 N*m of a 130 N*m limit
        # with 0 % clipping, so the hip is not torque-starved and a higher Kp
        # cannot create a gait. Pay for velocity instead, and make the hard
        # tracking term (which subtracts 0.2*error on top of the exp kernel)
        # worth twice as much so standing under a 0.4 command stops breaking even.
        class scales(X1DHStandCfg.rewards.scales):
            # exp_ada_25: stop paying for the hop. The inherited feet_air_time pays for
            # ANY lift, including both feet at once, and on F1 that became a two-foot
            # bounce: exp_ada_23 shows 5 cm of sole clearance 10-12 % of the time but
            # 9.9 % of it with BOTH feet up, while single support stays at 7.5 % and the
            # 14 falls are fore-aft topples (pitch 14-27 deg half a second before the
            # reset, |roll| <= 15 deg). Replace it with a real single-support air time
            # (paid only when a foot lands while the other foot is on the ground and the
            # flight lasted more than 2 control steps) and charge for double-airborne
            # steps. Scales are multiplied by dt = 0.01 s, so double_airborne = -0.5 is
            # -0.5 per second of flight: a 0.3-0.5 s hop costs 0.15-0.25 while one
            # genuine landing pays at most 0.02, i.e. hopping is now strictly worse than
            # standing and the velocity terms have to do the work of creating strides.
            feet_air_time = 0.
            feet_air_time_ss = 4.0
            double_airborne = -0.5
            foot_slip = -0.03
            feet_contact_forces = -0.004
            tracking_lin_vel = 2.6
            track_vel_hard = 1.0


class WeichaiF112CfgPPO(X1DHStandCfgPPO):
    class runner(X1DHStandCfgPPO.runner):
        experiment_name = 'weichai_f1_12_dh_stand'
