use core::f32::consts::{FRAC_PI_2, PI};
use kinematics::{Quaternion, Rotatable, Vector2, Vector3, VectorSpace};

fn close(actual: Vector3, expected: Vector3) {
    assert!(
        actual.sub(expected).length() < 2e-6,
        "{actual:?} != {expected:?}"
    );
}

#[test]
fn small_nonzero_vectors_keep_their_direction() {
    close(
        Vector3 {
            x: 1e-5,
            y: 0.0,
            z: 0.0,
        }
        .normalize(),
        Vector3::FORWARD,
    );
    assert_eq!(
        Vector2 { x: 0.0, y: 1e-5 }.normalize(),
        Vector2 { x: 0.0, y: 1.0 }
    );
}

#[test]
fn small_axes_still_define_a_rotation() {
    let rotation = Quaternion::from_axis_angle(
        Vector3 {
            x: 0.0,
            y: 0.0,
            z: 1e-5,
        },
        FRAC_PI_2,
    );
    close(
        rotation.rotate_vector(Vector3::FORWARD),
        Vector3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
    );
}

#[test]
fn rotation_arc_handles_opposite_and_zero_directions() {
    let opposite = Vector3 {
        x: -1.0,
        y: 0.0,
        z: 0.0,
    };
    close(
        Quaternion::from_rotation_arc(Vector3::FORWARD, opposite).rotate_vector(Vector3::FORWARD),
        opposite,
    );
    assert_eq!(
        Quaternion::from_rotation_arc(Vector3::ZERO, opposite),
        Quaternion::IDENTITY
    );
}

#[test]
fn quaternion_normalization_preserves_rotation_and_inverse() {
    let q = Quaternion {
        x: 0.0,
        y: 0.0,
        z: 2.0,
        w: 2.0,
    }
    .normalized();
    let turned = q.rotate_vector(Vector3::FORWARD);
    close(
        turned,
        Vector3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
    );
    close(q.conjugate().rotate_vector(turned), Vector3::FORWARD);
}

#[test]
fn planar_and_spatial_rotations_agree() {
    let p = Vector2 { x: 2.0, y: -1.0 }.rotate(PI / 3.0);
    let q = Quaternion::from_axis_angle(
        Vector3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        PI / 3.0,
    );
    close(
        q.rotate_vector(Vector3 {
            x: 2.0,
            y: -1.0,
            z: 0.0,
        }),
        Vector3 {
            x: p.x,
            y: p.y,
            z: 0.0,
        },
    );
}
