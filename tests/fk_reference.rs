use kinematics::{ArmState, Joint, JointConstraint, Quaternion, Vector3, VectorSpace};

type Matrix = [[f64; 3]; 3];

fn rotation_matrix(q: Quaternion) -> Matrix {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn multiply(a: Matrix, b: Matrix) -> Matrix {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

fn reference_fk<const N: usize>(state: &ArmState<N, Vector3>) -> [f64; 3] {
    let base = state.base_position;
    let mut position = [base.x, base.y, base.z].map(f64::from);
    let mut frame = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for joint in &state.joints {
        frame = multiply(
            multiply(frame, rotation_matrix(joint.base_rotation)),
            rotation_matrix(joint.rotation),
        );
        for (coordinate, row) in position.iter_mut().zip(frame) {
            *coordinate += row[0] * f64::from(joint.length);
        }
    }
    position
}

fn sample_rotation(seed: usize) -> Quaternion {
    let t = seed as f32;
    Quaternion {
        x: (0.7 * t).sin(),
        y: (1.1 * t).cos(),
        z: (0.3 * t).sin(),
        w: (0.5 * t).cos(),
    }
}

#[test]
fn fk_matches_an_independent_f64_matrix_model() {
    for scale in [0.001, 1.0, 1000.0] {
        for sample in 0..32 {
            let state = ArmState {
                joints: std::array::from_fn::<_, 8, _>(|index| Joint {
                    rotation: sample_rotation(sample * 8 + index),
                    base_rotation: sample_rotation(sample * 8 + index + 37),
                    length: index as f32 * scale,
                    constraint: JointConstraint::None,
                }),
                base_position: Vector3 {
                    x: 2.0 * scale,
                    y: -3.0 * scale,
                    z: 0.5 * scale,
                },
                effector_position: Vector3::ZERO,
            };
            let expected = reference_fk(&state);
            let actual = state.solve_fk();
            let error = [actual.x, actual.y, actual.z]
                .into_iter()
                .zip(expected)
                .map(|(a, b)| (f64::from(a) - b).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(
                error <= 1.0e-4 * f64::from(scale),
                "sample {sample}, scale {scale}: error {error}"
            );
        }
    }
}
