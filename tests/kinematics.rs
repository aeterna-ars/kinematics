use core::f32::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_6, PI};
use kinematics::{
    ArmState, BallConstraint, HingeConstraint, Joint, JointConstraint, MotorAxis, Quaternion,
    Vector3,
};

fn v(x: f32, y: f32, z: f32) -> Vector3 {
    Vector3 { x, y, z }
}

// Use std trigonometry so fixtures do not reuse the code under test.
fn rotation(axis: MotorAxis, angle: f32) -> Quaternion {
    let (s, c) = (angle * 0.5).sin_cos();
    match axis {
        MotorAxis::X => Quaternion {
            x: s,
            y: 0.0,
            z: 0.0,
            w: c,
        },
        MotorAxis::Y => Quaternion {
            x: 0.0,
            y: s,
            z: 0.0,
            w: c,
        },
        MotorAxis::Z => Quaternion {
            x: 0.0,
            y: 0.0,
            z: s,
            w: c,
        },
    }
}

fn link(length: f32) -> Joint<Vector3> {
    Joint {
        rotation: Quaternion::IDENTITY,
        length,
        base_rotation: Quaternion::IDENTITY,
        constraint: JointConstraint::None,
    }
}

fn hinge(length: f32, axis: MotorAxis, min_rad: f32, max_rad: f32) -> Joint<Vector3> {
    Joint {
        constraint: JointConstraint::Hinge(HingeConstraint {
            axis,
            min_rad,
            max_rad,
        }),
        ..link(length)
    }
}

fn arm<const N: usize>(joints: [Joint<Vector3>; N]) -> ArmState<N, Vector3> {
    ArmState {
        joints,
        base_position: v(0.0, 0.0, 0.0),
        effector_position: v(91.0, -37.0, 8.0),
    }
}

fn distance(a: Vector3, b: Vector3) -> f32 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

fn assert_position(actual: Vector3, expected: Vector3, tolerance: f32) {
    assert!(
        distance(actual, expected) <= tolerance,
        "expected {expected:?}, got {actual:?}; tolerance {tolerance}"
    );
}

fn assert_unit(q: Quaternion) {
    let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    assert!((norm - 1.0).abs() < 2.0e-5, "quaternion is not unit: {q:?}");
}

// Bitwise equality handles NaNs and detects signed-zero changes.
fn snapshot<const N: usize>(arm: &ArmState<N, Vector3>) -> Vec<u32> {
    let mut bits = Vec::new();
    for position in [arm.base_position, arm.effector_position] {
        bits.extend([position.x, position.y, position.z].map(f32::to_bits));
    }
    for joint in &arm.joints {
        bits.push(joint.length.to_bits());
        for q in [joint.rotation, joint.base_rotation] {
            bits.extend([q.x, q.y, q.z, q.w].map(f32::to_bits));
        }
        match joint.constraint {
            JointConstraint::None => bits.push(0),
            JointConstraint::Hinge(c) => {
                bits.extend([
                    1,
                    match c.axis {
                        MotorAxis::X => 0,
                        MotorAxis::Y => 1,
                        MotorAxis::Z => 2,
                    },
                    c.min_rad.to_bits(),
                    c.max_rad.to_bits(),
                ]);
            }
            JointConstraint::Ball(c) => bits.extend([2, c.max_cone_rad.to_bits()]),
        }
    }
    bits
}

#[test]
fn fk_sums_straight_links_and_ignores_stale_effector_cache() {
    let state = arm([link(1.0), link(2.0), link(0.5)]);
    let before = snapshot(&state);
    assert_position(state.solve_fk(), v(3.5, 0.0, 0.0), 1.0e-6);
    assert_eq!(snapshot(&state), before);
}

#[test]
fn fk_composes_each_joint_with_its_parent_orientation() {
    let mut state = arm([link(1.0), link(2.0)]);
    state.joints[0].rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    state.joints[1].rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    assert_position(state.solve_fk(), v(-2.0, 1.0, 0.0), 2.0e-6);
}

#[test]
fn fk_applies_local_rotation_before_rest_frame_rotation() {
    let mut state = arm([link(1.0)]);
    state.joints[0].base_rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    state.joints[0].rotation = rotation(MotorAxis::Y, FRAC_PI_2);
    // Y sends +X to -Z; the rest frame's Z rotation leaves -Z unchanged.
    assert_position(state.solve_fk(), v(0.0, 0.0, -1.0), 1.0e-6);
}

#[test]
fn fk_includes_the_world_base_translation() {
    let mut state = arm([link(2.0)]);
    state.base_position = v(7.0, -4.0, 3.0);
    state.joints[0].rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    assert_position(state.solve_fk(), v(7.0, -2.0, 3.0), 1.0e-6);
}

#[test]
fn empty_arm_has_its_effector_at_the_base() {
    let mut state = arm([]);
    state.base_position = v(2.0, 3.0, -4.0);
    assert_position(state.solve_fk(), state.base_position, 0.0);
    let report = state
        .solve_ik(20, 1.0e-4, v(2.0, 3.0, -4.0))
        .expect("valid empty arm");
    assert!(report.converged);
    assert_eq!(report.iterations, 0);
    assert_position(state.effector_position, state.base_position, 0.0);

    let report = state
        .solve_ik(20, 1.0e-4, v(2.0, 3.0, -2.0))
        .expect("valid unreachable target");
    assert!(!report.converged);
    assert!((report.distance - 2.0).abs() < 1.0e-6);
}

#[test]
fn fk_evaluates_the_pose_without_clamping_joint_limits() {
    let mut state = arm([hinge(1.0, MotorAxis::Z, 0.0, 0.0)]);
    state.joints[0].rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    let before = snapshot(&state);
    assert_position(state.solve_fk(), v(0.0, 1.0, 0.0), 1.0e-6);
    assert_eq!(snapshot(&state), before);
}

#[test]
fn ik_reaches_a_planar_two_link_target() {
    let mut state = arm([hinge(1.0, MotorAxis::Z, -PI, PI); 2]);
    let target = v(1.0, 1.0, 0.0);
    let report = state.solve_ik(128, 1.0e-4, target).expect("valid arm");
    assert!(report.converged);
    assert!(report.iterations <= 128);
    assert_position(state.solve_fk(), target, 1.0e-4);
    assert_position(state.effector_position, state.solve_fk(), 1.0e-6);
}

#[test]
fn ik_reaches_a_spatial_target_from_a_translated_base() {
    let mut state = arm([link(1.0); 3]);
    state.base_position = v(2.0, -1.0, 0.5);
    let target = v(2.6, 0.2, 1.6);
    let report = state.solve_ik(256, 1.0e-3, target).expect("valid arm");
    assert!(report.converged);
    assert_position(state.solve_fk(), target, 1.0e-3);
    assert!((report.distance - distance(state.solve_fk(), target)).abs() < 1.0e-5);
    for joint in &state.joints {
        assert_unit(joint.rotation);
    }
}

#[test]
fn ik_reports_an_unreachable_target_without_nonfinite_output() {
    let mut state = arm([link(1.0); 2]);
    let target = v(6.0, 2.0, 3.0);
    let report = state
        .solve_ik(80, 1.0e-4, target)
        .expect("unreachable is not invalid");
    assert!(!report.converged);
    assert!(report.iterations <= 80);
    assert!(report.distance.is_finite());
    assert!(report.distance >= 5.0 - 1.0e-5);
    assert!(distance(state.effector_position, state.base_position) <= 2.0 + 1.0e-5);
    assert_position(state.effector_position, state.solve_fk(), 1.0e-5);
    for joint in &state.joints {
        assert_unit(joint.rotation);
    }
}

#[test]
fn ik_can_turn_a_single_link_to_its_antiparallel_target() {
    let mut state = arm([link(1.0)]);
    let target = v(-1.0, 0.0, 0.0);
    let report = state.solve_ik(8, 1.0e-4, target).expect("valid arm");
    assert!(report.converged);
    assert_position(state.solve_fk(), target, 1.0e-4);
}

#[test]
fn ik_preserves_small_angular_corrections_for_long_links() {
    let mut state = arm([link(1.0e6)]);
    let target = v(1.0e6, 0.05, 0.0);
    let report = state.solve_ik(1, 0.001, target).expect("valid arm");
    assert!(
        report.converged,
        "a representable nonzero correction was lost: distance {}",
        report.distance
    );
    assert_position(state.solve_fk(), target, 0.001);
}

#[test]
fn ik_escapes_a_straight_chain_for_an_inside_reach_collinear_target() {
    let mut state = arm([hinge(1.0, MotorAxis::Z, -PI, PI); 2]);
    let target = v(0.1, 0.0, 0.0);
    let report = state.solve_ik(512, 1.0e-3, target).expect("valid arm");
    assert!(
        report.converged,
        "collinear reachable target stalled at distance {}",
        report.distance
    );
    assert_position(state.solve_fk(), target, 1.0e-3);
}

#[test]
fn ik_respects_a_z_hinge_limit_when_the_target_is_outside_it() {
    let mut state = arm([hinge(1.0, MotorAxis::Z, -FRAC_PI_6, FRAC_PI_6)]);
    let report = state
        .solve_ik(16, 1.0e-4, v(0.0, 1.0, 0.0))
        .expect("valid arm");
    assert!(!report.converged);
    let q = state.joints[0].rotation;
    assert!(q.x.abs() < 1.0e-6 && q.y.abs() < 1.0e-6);
    let angle = 2.0 * q.z.atan2(q.w);
    assert!((-FRAC_PI_6 - 1.0e-5..=FRAC_PI_6 + 1.0e-5).contains(&angle));
    assert_position(
        state.solve_fk(),
        v(FRAC_PI_6.cos(), FRAC_PI_6.sin(), 0.0),
        1.0e-4,
    );
}

#[test]
fn ik_rotates_about_a_y_hinge_axis() {
    let mut state = arm([hinge(1.0, MotorAxis::Y, -PI, PI)]);
    let target = v(0.0, 0.0, -1.0);
    let report = state.solve_ik(16, 1.0e-4, target).expect("valid arm");
    assert!(report.converged);
    assert_position(state.solve_fk(), target, 1.0e-4);
    let q = state.joints[0].rotation;
    assert!(q.x.abs() < 1.0e-6 && q.z.abs() < 1.0e-6);
}

#[test]
fn ik_rotates_about_an_x_hinge_to_move_an_offset_child() {
    let mut state = arm([
        hinge(1.0, MotorAxis::X, -PI, PI),
        hinge(1.0, MotorAxis::Z, 0.0, 0.0),
    ]);
    state.joints[1].base_rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    let target = v(1.0, 0.0, 1.0);
    let report = state.solve_ik(32, 1.0e-4, target).expect("valid arm");
    assert!(report.converged);
    assert_position(state.solve_fk(), target, 1.0e-4);
    let q = state.joints[0].rotation;
    assert!(q.y.abs() < 1.0e-6 && q.z.abs() < 1.0e-6);
    assert_position(
        state.joints[1].rotation.rotate_vector(v(1.0, 0.0, 0.0)),
        v(1.0, 0.0, 0.0),
        1.0e-6,
    );
}

#[test]
fn ik_interprets_the_hinge_axis_in_its_rotated_rest_frame() {
    let mut state = arm([hinge(1.0, MotorAxis::Z, -PI, PI)]);
    state.joints[0].base_rotation = rotation(MotorAxis::X, FRAC_PI_2);
    let target = v(0.0, 0.0, 1.0);
    let report = state.solve_ik(16, 1.0e-4, target).expect("valid arm");
    assert!(report.converged);
    assert_position(state.solve_fk(), target, 1.0e-4);
    let q = state.joints[0].rotation;
    assert!(q.x.abs() < 1.0e-6 && q.y.abs() < 1.0e-6);
}

#[test]
fn ik_keeps_a_ball_joint_inside_its_local_cone() {
    let mut state = arm([link(1.0)]);
    state.joints[0].base_rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    state.joints[0].rotation = rotation(MotorAxis::Y, FRAC_PI_2);
    state.joints[0].constraint = JointConstraint::Ball(BallConstraint {
        max_cone_rad: FRAC_PI_6,
    });
    let report = state
        .solve_ik(48, 1.0e-4, v(-1.0, 0.0, 0.0))
        .expect("valid ball joint");
    assert!(!report.converged);
    let local_forward = state.joints[0].rotation.rotate_vector(v(1.0, 0.0, 0.0));
    assert!(local_forward.x >= FRAC_PI_6.cos() - 2.0e-5);
    assert_unit(state.joints[0].rotation);
    assert_position(state.effector_position, state.solve_fk(), 1.0e-6);
}

#[test]
fn ball_projection_preserves_the_plane_of_a_nearly_opposite_swing() {
    let mut state = arm([link(1.0)]);
    // Nonzero w preserves the swing plane just short of a half turn.
    state.joints[0].rotation = Quaternion {
        x: 0.0,
        y: 1.0,
        z: 0.0,
        w: 2.5e-8,
    };
    state.joints[0].constraint = JointConstraint::Ball(BallConstraint {
        max_cone_rad: FRAC_PI_6,
    });
    let expected = v(FRAC_PI_6.cos(), 0.0, -FRAC_PI_6.sin());
    state
        .solve_ik(0, 1.0e-5, expected)
        .expect("valid ball joint");
    assert_position(state.effector_position, expected, 1.0e-5);
}

#[test]
fn a_zero_ball_cone_preserves_unrestricted_twist() {
    let mut state = arm([link(1.0)]);
    state.joints[0].rotation = rotation(MotorAxis::X, FRAC_PI_2);
    state.joints[0].constraint = JointConstraint::Ball(BallConstraint { max_cone_rad: 0.0 });
    let report = state
        .solve_ik(0, 1.0e-5, v(1.0, 0.0, 0.0))
        .expect("valid ball joint");
    assert!(report.converged);
    assert_eq!(report.iterations, 0);
    assert_position(
        state.joints[0].rotation.rotate_vector(v(0.0, 1.0, 0.0)),
        v(0.0, 0.0, 1.0),
        1.0e-5,
    );
}

#[test]
fn a_zero_ball_cone_can_twist_an_offset_child_through_a_half_turn() {
    let mut state = arm([link(0.0), hinge(1.0, MotorAxis::Z, 0.0, 0.0)]);
    state.joints[0].constraint = JointConstraint::Ball(BallConstraint { max_cone_rad: 0.0 });
    state.joints[1].base_rotation = rotation(MotorAxis::Z, FRAC_PI_2);
    let target = v(0.0, -1.0, 0.0);
    let report = state
        .solve_ik(32, 1.0e-4, target)
        .expect("valid ball joint");
    assert!(
        report.converged,
        "free twist must remain usable with a zero cone"
    );
    assert_position(state.solve_fk(), target, 1.0e-4);
}

#[test]
fn arithmetic_overflow_leaves_the_entire_pose_unchanged() {
    let mut state = arm([link(f32::MAX); 2]);
    let before = snapshot(&state);
    assert_eq!(
        state.solve_ik(32, 0.01, v(0.0, 0.0, 0.0)),
        Err(kinematics::IkError::NumericalOverflow)
    );
    assert_eq!(snapshot(&state), before);
}

#[test]
fn overflow_after_a_valid_sweep_preserves_every_field() {
    // A nonunit input makes a premature pose commit visible.
    let mut state = arm([Joint {
        rotation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 2.0,
            w: 0.0,
        },
        ..link(f32::MAX)
    }]);
    state.base_position = v(f32::MAX, 0.0, 0.0);
    state.effector_position = v(-0.0, f32::from_bits(0x7fc1_2345), f32::NEG_INFINITY);
    let target = v(f32::MAX * 0.5, 0.0, 0.0);
    let before = snapshot(&state);
    assert_eq!(state.solve_fk(), v(0.0, 0.0, 0.0));

    let mut one_sweep = arm(state.joints);
    one_sweep.base_position = state.base_position;
    let report = one_sweep.solve_ik(1, 0.0, target).unwrap();
    assert_eq!(report.iterations, 1);
    assert!(!report.converged);
    assert_ne!(one_sweep.joints[0].rotation, state.joints[0].rotation);

    assert_eq!(
        state.solve_ik(8, 0.0, target),
        Err(kinematics::IkError::NumericalOverflow)
    );
    assert_eq!(snapshot(&state), before);
}

#[test]
fn zero_iterations_normalize_project_limits_and_refresh_the_cache() {
    let mut state = arm([hinge(1.0, MotorAxis::Z, FRAC_PI_6, FRAC_PI_6)]);
    let q = rotation(MotorAxis::Z, FRAC_PI_2);
    state.joints[0].rotation = Quaternion {
        x: q.x * 3.0,
        y: q.y * 3.0,
        z: q.z * 3.0,
        w: q.w * 3.0,
    };
    let target = v(0.0, 3.0, 0.0);
    let report = state
        .solve_ik(0, 1.0e-4, target)
        .expect("valid nonunit quaternion");
    assert_eq!(report.iterations, 0);
    assert!(!report.converged);
    assert_unit(state.joints[0].rotation);
    assert_position(
        state.effector_position,
        v(FRAC_PI_6.cos(), FRAC_PI_6.sin(), 0.0),
        1.0e-5,
    );
    assert!((report.distance - distance(state.effector_position, target)).abs() < 1.0e-5);
}

#[test]
fn zero_budget_preserves_reached_hinge_pose_for_both_quaternion_signs() {
    for axis in [MotorAxis::Y, MotorAxis::Z] {
        for angle in [-5e-8, 5e-8] {
            let target = match axis {
                MotorAxis::Y => v(1.0e6, 0.0, -angle * 1.0e6),
                _ => v(1.0e6, angle * 1.0e6, 0.0),
            };
            let q = rotation(axis, angle);
            for sign in [-1.0, 1.0] {
                let mut state = arm([hinge(1.0e6, axis, -PI, PI)]);
                state.joints[0].rotation = Quaternion {
                    x: sign * q.x,
                    y: sign * q.y,
                    z: sign * q.z,
                    w: sign * q.w,
                };
                assert_position(state.solve_fk(), target, 1.0e-6);

                let report = state.solve_ik(0, 0.001, target).unwrap();
                assert!(
                    report.converged,
                    "{axis:?}, angle={angle}, sign={sign}: {report:?}"
                );
                assert_eq!(report.iterations, 0);
                assert_position(state.solve_fk(), target, 1.0e-6);
                assert_position(state.effector_position, target, 1.0e-6);
            }
        }
    }
}

#[test]
fn an_already_reached_target_uses_no_iterations() {
    let mut state = arm([link(1.0); 2]);
    let report = state
        .solve_ik(100, 0.0, v(2.0, 0.0, 0.0))
        .expect("valid arm");
    assert!(report.converged);
    assert_eq!(report.iterations, 0);
    assert_eq!(report.distance, 0.0);
    assert_position(state.effector_position, v(2.0, 0.0, 0.0), 0.0);
}

#[test]
fn a_zero_length_joint_can_orient_a_fixed_child() {
    let mut state = arm([link(0.0), hinge(1.0, MotorAxis::Z, 0.0, 0.0)]);
    let target = v(0.0, 1.0, 0.0);
    let report = state
        .solve_ik(32, 1.0e-4, target)
        .expect("zero-length link is valid");
    assert!(report.converged);
    assert_position(state.solve_fk(), target, 1.0e-4);
    assert_eq!(state.joints[0].length, 0.0);
    assert_eq!(state.joints[1].length, 1.0);
    assert_position(
        state.joints[1].rotation.rotate_vector(v(1.0, 0.0, 0.0)),
        v(1.0, 0.0, 0.0),
        1.0e-6,
    );
}

#[test]
fn ik_honors_the_iteration_budget_and_reports_the_final_pose_distance() {
    let mut state = arm([link(1.0); 3]);
    let target = v(0.3, 0.7, 1.1);
    let report = state.solve_ik(1, 1.0e-6, target).expect("valid arm");
    assert!(report.iterations <= 1);
    assert_position(state.effector_position, state.solve_fk(), 1.0e-6);
    let actual_distance = distance(state.effector_position, target);
    assert!((report.distance - actual_distance).abs() < 1.0e-6);
    assert_eq!(report.converged, actual_distance <= 1.0e-6);
}

#[test]
fn more_iterations_preserve_or_improve_the_best_pose() {
    let joints = [
        hinge(1.0, MotorAxis::Z, -FRAC_PI_3, FRAC_PI_3),
        Joint {
            constraint: JointConstraint::Ball(BallConstraint {
                max_cone_rad: FRAC_PI_6,
            }),
            base_rotation: rotation(MotorAxis::X, FRAC_PI_3),
            ..link(0.75)
        },
        hinge(0.5, MotorAxis::Y, -FRAC_PI_2, FRAC_PI_2),
    ];
    for target in [v(1.0, 0.0, 0.0), v(0.4, -0.8, 1.2), v(3.0, 0.0, 0.0)] {
        let mut best_distance = f32::INFINITY;
        for budget in [0, 1, 2, 3, 4, 8, 16, 32] {
            let mut state = arm(joints);
            let report = state.solve_ik(budget, 1.0e-5, target).unwrap();
            let actual_distance = distance(state.solve_fk(), target);
            assert!(
                actual_distance <= best_distance + 2.0e-6,
                "target {target:?}, budget {budget}: {actual_distance} > {best_distance}"
            );
            assert!(report.iterations <= budget);
            assert!((report.distance - actual_distance).abs() < 2.0e-6);
            assert_eq!(state.effector_position, state.solve_fk());
            best_distance = actual_distance;
        }
    }
}

#[test]
fn invalid_targets_and_tolerances_leave_every_field_unchanged() {
    let cases = [
        (v(f32::NAN, 0.0, 0.0), 0.01),
        (v(0.0, f32::INFINITY, 0.0), 0.01),
        (v(0.0, 0.0, f32::NEG_INFINITY), 0.01),
        (v(1.0, 1.0, 0.0), -0.01),
        (v(1.0, 1.0, 0.0), f32::NAN),
        (v(1.0, 1.0, 0.0), f32::INFINITY),
    ];
    for (target, tolerance) in cases {
        let mut state = arm([hinge(1.0, MotorAxis::Z, 0.0, FRAC_PI_3); 2]);
        state.joints[0].rotation = rotation(MotorAxis::Z, -FRAC_PI_2);
        let before = snapshot(&state);
        assert!(state.solve_ik(32, tolerance, target).is_err());
        assert_eq!(
            snapshot(&state),
            before,
            "validation must precede all pose changes"
        );
    }
}

#[test]
fn invalid_positions_and_link_lengths_leave_the_state_unchanged() {
    for value in [-1.0, f32::NAN, f32::INFINITY] {
        let mut state = arm([hinge(1.0, MotorAxis::Z, FRAC_PI_6, FRAC_PI_6); 2]);
        state.joints[1].length = value;
        let before = snapshot(&state);
        assert!(state.solve_ik(32, 0.01, v(1.0, 1.0, 0.0)).is_err());
        assert_eq!(snapshot(&state), before);
    }
    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut state = arm([link(1.0)]);
        state.base_position.y = value;
        let before = snapshot(&state);
        assert!(state.solve_ik(32, 0.01, v(1.0, 1.0, 0.0)).is_err());
        assert_eq!(snapshot(&state), before);
    }
}

#[test]
fn zero_or_nonfinite_quaternions_are_rejected_without_partial_mutation() {
    let invalid = [
        Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
        Quaternion {
            x: f32::NAN,
            ..Quaternion::IDENTITY
        },
        Quaternion {
            y: f32::INFINITY,
            ..Quaternion::IDENTITY
        },
    ];
    for q in invalid {
        for replace_base in [false, true] {
            let mut state = arm([hinge(1.0, MotorAxis::Z, FRAC_PI_6, FRAC_PI_6); 2]);
            if replace_base {
                state.joints[1].base_rotation = q;
            } else {
                state.joints[1].rotation = q;
            }
            let before = snapshot(&state);
            assert!(state.solve_ik(32, 0.01, v(1.0, 1.0, 0.0)).is_err());
            assert_eq!(snapshot(&state), before);
        }
    }
}

#[test]
fn invalid_joint_limits_are_rejected_without_partial_mutation() {
    let invalid = [
        JointConstraint::Hinge(HingeConstraint {
            axis: MotorAxis::Z,
            min_rad: 1.0,
            max_rad: -1.0,
        }),
        JointConstraint::Hinge(HingeConstraint {
            axis: MotorAxis::Y,
            min_rad: -PI - 0.1,
            max_rad: PI,
        }),
        JointConstraint::Hinge(HingeConstraint {
            axis: MotorAxis::X,
            min_rad: -PI,
            max_rad: PI + 0.1,
        }),
        JointConstraint::Hinge(HingeConstraint {
            axis: MotorAxis::X,
            min_rad: f32::NAN,
            max_rad: PI,
        }),
        JointConstraint::Hinge(HingeConstraint {
            axis: MotorAxis::Y,
            min_rad: -PI,
            max_rad: f32::INFINITY,
        }),
        JointConstraint::Ball(BallConstraint { max_cone_rad: -0.1 }),
        JointConstraint::Ball(BallConstraint {
            max_cone_rad: PI + 0.1,
        }),
        JointConstraint::Ball(BallConstraint {
            max_cone_rad: f32::NAN,
        }),
        JointConstraint::Ball(BallConstraint {
            max_cone_rad: f32::INFINITY,
        }),
    ];
    for constraint in invalid {
        let mut state = arm([hinge(1.0, MotorAxis::Z, FRAC_PI_6, FRAC_PI_6); 2]);
        state.joints[1].constraint = constraint;
        let before = snapshot(&state);
        assert!(state.solve_ik(32, 0.01, v(1.0, 1.0, 0.0)).is_err());
        assert_eq!(snapshot(&state), before);
    }
}

#[test]
fn ik_reaches_a_collinear_target_using_small_constrained_rotations() {
    let length = 1.0e6_f32;
    let bend = 0.001_f32;
    let tolerance = 0.25;
    let target = v(1_999_999.0, 0.0, 0.0);
    let joints = [
        hinge(length, MotorAxis::Z, -bend, bend),
        hinge(length, MotorAxis::Z, -2.0 * bend, 2.0 * bend),
    ];

    // Angles +bend and -2*bend give x = 2L*cos(bend), y = 0.
    let analytical_target = v(
        (2.0 * f64::from(length) * f64::from(bend).cos()) as f32,
        0.0,
        0.0,
    );
    assert_position(analytical_target, target, tolerance);
    let mut witness = arm(joints);
    witness.joints[0].rotation = rotation(MotorAxis::Z, bend);
    witness.joints[1].rotation = rotation(MotorAxis::Z, -2.0 * bend);
    assert_position(witness.solve_fk(), target, tolerance);

    let mut state = arm(joints);
    let report = state
        .solve_ik(128, tolerance, target)
        .expect("valid arm with a reachable target");
    assert!(
        report.converged,
        "reachable target stalled at distance {} after {} iterations; position {:?}",
        report.distance,
        report.iterations,
        state.solve_fk()
    );
    assert_position(state.solve_fk(), target, tolerance);
}

#[test]
fn ik_restarts_past_a_joint_with_only_roundoff_changes() {
    let bend = 0.001;
    let tolerance = 0.25;
    let target = v(1_999_999.0, 0.0, 0.0);
    for twist in [0.1, 2.0e-40] {
        let joints = [
            Joint {
                rotation: rotation(MotorAxis::X, twist),
                constraint: JointConstraint::Ball(BallConstraint { max_cone_rad: 0.0 }),
                ..link(0.0)
            },
            hinge(1.0e6, MotorAxis::Z, -bend, bend),
            hinge(1.0e6, MotorAxis::Z, -2.0 * bend, 2.0 * bend),
        ];
        let mut witness = arm(joints);
        witness.joints[1].rotation = rotation(MotorAxis::Z, bend);
        witness.joints[2].rotation = rotation(MotorAxis::Z, -2.0 * bend);
        assert_position(witness.solve_fk(), target, tolerance);

        let mut state = arm(joints);
        let report = state.solve_ik(128, tolerance, target).unwrap();
        assert!(report.converged, "twist {twist}: {report:?}");
        assert_position(state.solve_fk(), target, tolerance);
    }
}

#[test]
fn ik_restarts_on_later_joints_to_bend_a_straight_chain() {
    let joints = [
        hinge(1.0, MotorAxis::X, -PI, PI),
        hinge(1.0, MotorAxis::Z, -PI, PI),
        hinge(1.0, MotorAxis::Z, -PI, PI),
    ];
    let target = v(1.0 + 3.0_f32.sqrt(), 0.0, 0.0);
    let tolerance = 1.0e-4;

    // World angles 0, +pi/6, -pi/6 give x = 1 + sqrt(3), y = 0.
    let mut witness = arm(joints);
    witness.joints[1].rotation = rotation(MotorAxis::Z, FRAC_PI_6);
    witness.joints[2].rotation = rotation(MotorAxis::Z, -FRAC_PI_3);
    assert_position(witness.solve_fk(), target, tolerance);
    assert!(witness.solve_ik(0, tolerance, target).unwrap().converged);

    let mut state = arm(joints);
    let report = state.solve_ik(128, tolerance, target).unwrap();
    assert!(report.converged, "{report:?}");
    assert_position(state.solve_fk(), target, tolerance);
}

#[test]
fn ik_restarts_in_other_directions_when_z_seeds_only_twist_the_chain() {
    fn check<const N: usize>() {
        let joints = std::array::from_fn(|index| Joint {
            rotation: rotation(MotorAxis::Y, -FRAC_PI_2),
            base_rotation: if index == 0 {
                Quaternion::IDENTITY
            } else {
                rotation(MotorAxis::Y, FRAC_PI_2)
            },
            ..link(1.0)
        });
        let target = v(0.0, 0.0, N as f32 - 0.5);
        let tolerance = 1.0e-4;
        let bend = 0.75_f32.acos();

        // The tilted end links give z = N - 2 + 2*cos(bend) = N - 0.5.
        let mut witness: ArmState<N, Vector3> = arm(joints);
        witness.joints[0].rotation = rotation(MotorAxis::Y, -FRAC_PI_2 + bend);
        witness.joints[1].rotation = rotation(MotorAxis::Y, -FRAC_PI_2 - bend);
        witness.joints[N - 1].rotation = rotation(MotorAxis::Y, -FRAC_PI_2 - bend);
        assert_position(witness.solve_fk(), target, 2.0e-5);

        let mut state = arm(joints);
        let report = state.solve_ik(128, tolerance, target).unwrap();
        assert!(report.converged, "{N} joints: {report:?}");
        assert!(report.iterations <= 128);
        assert_position(state.solve_fk(), target, tolerance);
    }

    check::<3>();
    check::<6>();
}
