use core::f32::consts::{FRAC_PI_2, PI};
use kinematics::*;

const X: Vector3 = Vector3 {
    x: 1.0,
    y: 0.0,
    z: 0.0,
};
const Y: Vector3 = Vector3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};
const Z: Vector3 = Vector3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

#[test]
fn hinge_projects_onto_axis_and_clamps_angle() {
    let limit = JointConstraint::Hinge(HingeConstraint {
        axis: MotorAxis::Z,
        min_rad: -0.25,
        max_rad: 0.5,
    });
    let input = Quaternion::from_axis_angle(Z, 1.0).mul_quat(Quaternion::from_axis_angle(X, 0.4));
    let actual = limit.apply_to_rotation(input);
    let expected = Quaternion::from_axis_angle(Z, 0.5);
    assert!(
        actual
            .rotate_vector(X)
            .sub(expected.rotate_vector(X))
            .length()
            < 1e-6
    );
    assert!(actual.rotate_vector(Z).sub(Z).length() < 1e-6);
}

#[test]
fn hinge_handles_the_pi_branch_cut_and_quaternion_sign() {
    for (axis, probe) in [(MotorAxis::X, Y), (MotorAxis::Y, Z), (MotorAxis::Z, X)] {
        let direction = axis.vector();
        for (min_rad, max_rad) in [(0.0, PI), (-PI, 0.0)] {
            let limit = JointConstraint::Hinge(HingeConstraint {
                axis,
                min_rad,
                max_rad,
            });
            for w in [
                -1e-7,
                -f32::from_bits(1),
                -0.0,
                0.0,
                f32::from_bits(1),
                1e-7,
            ] {
                for sign in [-1.0, 1.0] {
                    let q = Quaternion {
                        x: sign * direction.x,
                        y: sign * direction.y,
                        z: sign * direction.z,
                        w: sign * w,
                    };
                    let actual = limit.apply_to_rotation(q).rotate_vector(probe);
                    assert!(
                        actual.sub(probe.scale(-1.0)).length() < 5e-7,
                        "{axis:?}, limits [{min_rad}, {max_rad}], q={q:?}: {actual:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn hinge_preserves_small_angles_for_both_quaternion_signs() {
    for axis in [MotorAxis::X, MotorAxis::Y, MotorAxis::Z] {
        let direction = axis.vector();
        for angle in [-5e-8_f32, 5e-8] {
            let (sine, cosine) = (angle * 0.5).sin_cos();
            for (min_rad, max_rad) in [(-PI, PI), (angle - 1e-8, angle + 1e-8)] {
                let limit = JointConstraint::Hinge(HingeConstraint {
                    axis,
                    min_rad,
                    max_rad,
                });
                for sign in [-1.0, 1.0] {
                    let q = Quaternion {
                        x: sign * sine * direction.x,
                        y: sign * sine * direction.y,
                        z: sign * sine * direction.z,
                        w: sign * cosine,
                    };
                    let projected = limit.apply_to_rotation(q);
                    let component = match axis {
                        MotorAxis::X => projected.x,
                        MotorAxis::Y => projected.y,
                        MotorAxis::Z => projected.z,
                    };
                    let actual = 2.0 * f64::from(component).atan2(f64::from(projected.w));
                    assert!(
                        (actual - f64::from(angle)).abs() < 1e-12,
                        "{axis:?}, limits [{min_rad}, {max_rad}], q={q:?}: {actual}"
                    );
                }
            }
        }
    }
}

#[test]
fn ball_limits_swing_but_preserves_twist() {
    let twist = Quaternion::from_axis_angle(X, 0.7);
    let rotation = Quaternion::from_axis_angle(Y, 1.0).mul_quat(twist);
    let actual =
        JointConstraint::Ball(BallConstraint { max_cone_rad: 0.3 }).apply_to_rotation(rotation);
    let expected = Quaternion::from_axis_angle(Y, 0.3).mul_quat(twist);
    assert!(
        actual
            .rotate_vector(X)
            .sub(expected.rotate_vector(X))
            .length()
            < 2e-6
    );
    assert!(
        actual
            .rotate_vector(Z)
            .sub(expected.rotate_vector(Z))
            .length()
            < 2e-6
    );
}

#[test]
fn ball_handles_opposite_direction_and_zero_cone() {
    let q = Quaternion::from_axis_angle(Y, PI);
    let actual = JointConstraint::Ball(BallConstraint {
        max_cone_rad: FRAC_PI_2,
    })
    .apply_to_rotation(q);
    assert!(actual.rotate_vector(X).x.abs() < 2e-6);
    let locked = JointConstraint::Ball(BallConstraint { max_cone_rad: 0.0 }).apply_to_rotation(q);
    assert!(locked.rotate_vector(X).sub(X).length() < 2e-6);
}
