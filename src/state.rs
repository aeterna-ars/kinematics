use crate::constraint::*;
use crate::space::*;
use core::f32::consts::FRAC_PI_2;
use libm::atan2f;

const MAX_RESTARTS: usize = 6;
const RESTARTS: [(MotorAxis, f32); 6] = [
    (MotorAxis::Z, FRAC_PI_2),
    (MotorAxis::Z, -FRAC_PI_2),
    (MotorAxis::Y, FRAC_PI_2),
    (MotorAxis::Y, -FRAC_PI_2),
    (MotorAxis::X, FRAC_PI_2),
    (MotorAxis::X, -FRAC_PI_2),
];

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Joint<V: Rotatable> {
    pub rotation: V::Rotation,
    pub length: f32,
    pub base_rotation: Quaternion,
    pub constraint: JointConstraint,
}

pub struct ArmState<const JOINTS: usize, V: Rotatable> {
    pub joints: [Joint<V>; JOINTS],
    pub base_position: V,
    pub effector_position: V,
}

/// Result of a position-only IK solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkReport {
    /// Whether the returned pose is within the requested tolerance.
    pub converged: bool,
    /// CCD sweeps performed, at most `max_iters`.
    pub iterations: usize,
    /// Distance from the returned pose to the target.
    pub distance: f32,
}

/// Invalid input or overflow; the arm remains unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IkError {
    InvalidTarget,
    InvalidTolerance,
    InvalidBasePosition,
    InvalidLength { joint: usize },
    InvalidRotation { joint: usize },
    InvalidBaseRotation { joint: usize },
    InvalidConstraint { joint: usize },
    NumericalOverflow,
}

impl<const JOINTS: usize> ArmState<JOINTS, Vector3> {
    /// End position of the current pose, without mutation or limit projection.
    pub fn solve_fk(&self) -> Vector3 {
        let mut position = self.base_position;
        let mut orientation = Quaternion::IDENTITY;
        for joint in &self.joints {
            orientation = orientation
                .mul_quat(joint.base_rotation.normalized())
                .mul_quat(joint.rotation.normalized())
                .normalized();
            position = position.add(
                orientation
                    .rotate_vector(Vector3::FORWARD)
                    .scale(joint.length),
            );
        }
        position
    }

    /// Keep the closest constrained pose found and refresh the effector cache.
    /// A zero budget still applies limits. Errors leave the arm unchanged.
    /// Nonconvergence returns `Ok` with `converged: false`.
    pub fn solve_ik(
        &mut self,
        max_iters: usize,
        tolerance: f32,
        target: Vector3,
    ) -> Result<IkReport, IkError> {
        self.validate(tolerance, target)?;
        // Work on a copy so errors leave the caller's state intact.
        let mut pose = Self {
            joints: self.joints,
            base_position: self.base_position,
            effector_position: self.effector_position,
        };
        for joint in &mut pose.joints {
            joint.rotation = joint.constraint.apply_to_rotation(joint.rotation);
        }
        let mut distance = pose.refresh_distance(target)?;
        let mut best_distance = distance;
        let mut best_rotations = pose.joints.map(|joint| joint.rotation);
        let mut iterations = 0;
        // Stagger directions so the budget covers both axes and joints.
        let mut restart_candidates = (0..RESTARTS.len()).flat_map(|offset| {
            (0..JOINTS)
                .map(move |joint| (joint, (joint % RESTARTS.len() + offset) % RESTARTS.len()))
        });
        let mut restarts = 0;

        while distance > tolerance && iterations < max_iters && JOINTS != 0 {
            let before = distance;
            distance = pose.ccd_sweep(target, distance)?;
            iterations += 1;
            if distance < best_distance {
                best_distance = distance;
                best_rotations = pose.joints.map(|joint| joint.rotation);
            }
            if distance <= tolerance || iterations == max_iters {
                break;
            }
            if before - distance <= before * (8.0 * f32::EPSILON) {
                // Collinear joints can stall CCD even for reachable targets.
                pose.set_rotations(best_rotations);
                if restarts == MAX_RESTARTS
                    || !restart_candidates.any(|(joint, seed)| pose.perturb(joint, seed))
                {
                    break;
                }
                restarts += 1;
                distance = pose.refresh_distance(target)?;
                if distance < best_distance {
                    best_distance = distance;
                    best_rotations = pose.joints.map(|joint| joint.rotation);
                }
            }
        }

        pose.set_rotations(best_rotations);
        distance = pose.refresh_distance(target)?;
        self.joints = pose.joints;
        self.effector_position = pose.effector_position;
        Ok(IkReport {
            converged: distance <= tolerance,
            iterations,
            distance,
        })
    }

    fn ccd_sweep(&mut self, target: Vector3, mut distance: f32) -> Result<f32, IkError> {
        let (pivots, rest_frames) = self.joint_frames()?;
        // Child rotations do not affect the cached ancestor frames.
        for index in (0..JOINTS).rev() {
            let from = self.effector_position.sub(pivots[index]);
            let to = target.sub(pivots[index]);
            if !from.is_finite() || !to.is_finite() {
                return Err(IkError::NumericalOverflow);
            }
            let inverse_rest = rest_frames[index].conjugate();
            let from = inverse_rest.rotate_vector(from.normalize());
            let to = inverse_rest.rotate_vector(to.normalize());
            let constraint = self.joints[index].constraint;
            let delta = match constraint {
                JointConstraint::Hinge(hinge) => {
                    let axis = hinge.axis.vector();
                    let from = from.sub(axis.scale(from.dot(axis))).normalize();
                    let to = to.sub(axis.scale(to.dot(axis))).normalize();
                    let angle = atan2f(axis.dot(from.cross(to)), from.dot(to));
                    Quaternion::from_axis_angle(axis, angle)
                }
                _ => Quaternion::from_rotation_arc(from, to),
            };
            let previous_rotation = self.joints[index].rotation;
            let previous_effector = self.effector_position;
            self.joints[index].rotation =
                constraint.apply_to_rotation(delta.mul_quat(previous_rotation));
            let candidate_distance = self.refresh_distance(target)?;
            if candidate_distance <= distance {
                distance = candidate_distance;
            } else {
                self.joints[index].rotation = previous_rotation;
                self.effector_position = previous_effector;
            }
        }
        Ok(distance)
    }

    fn validate(&self, tolerance: f32, target: Vector3) -> Result<(), IkError> {
        if !target.is_finite() {
            return Err(IkError::InvalidTarget);
        }
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(IkError::InvalidTolerance);
        }
        if !self.base_position.is_finite() {
            return Err(IkError::InvalidBasePosition);
        }
        for (index, joint) in self.joints.iter().enumerate() {
            if !joint.length.is_finite() || joint.length < 0.0 {
                return Err(IkError::InvalidLength { joint: index });
            }
            if !joint.rotation.is_valid() {
                return Err(IkError::InvalidRotation { joint: index });
            }
            if !joint.base_rotation.is_valid() {
                return Err(IkError::InvalidBaseRotation { joint: index });
            }
            if !joint.constraint.is_valid() {
                return Err(IkError::InvalidConstraint { joint: index });
            }
        }
        Ok(())
    }

    fn refresh_distance(&mut self, target: Vector3) -> Result<f32, IkError> {
        self.effector_position = self.solve_fk();
        let distance = self.effector_position.sub(target).length();
        if !self.effector_position.is_finite() || !distance.is_finite() {
            return Err(IkError::NumericalOverflow);
        }
        Ok(distance)
    }

    fn joint_frames(&self) -> Result<([Vector3; JOINTS], [Quaternion; JOINTS]), IkError> {
        let mut pivots = [Vector3::ZERO; JOINTS];
        let mut rest_frames = [Quaternion::IDENTITY; JOINTS];
        let mut position = self.base_position;
        let mut orientation = Quaternion::IDENTITY;
        for (index, joint) in self.joints.iter().enumerate() {
            pivots[index] = position;
            rest_frames[index] = orientation
                .mul_quat(joint.base_rotation.normalized())
                .normalized();
            orientation = rest_frames[index].mul_quat(joint.rotation).normalized();
            position = position.add(
                orientation
                    .rotate_vector(Vector3::FORWARD)
                    .scale(joint.length),
            );
            if !position.is_finite() {
                return Err(IkError::NumericalOverflow);
            }
        }
        Ok((pivots, rest_frames))
    }

    fn set_rotations(&mut self, rotations: [Quaternion; JOINTS]) {
        for (joint, rotation) in self.joints.iter_mut().zip(rotations) {
            joint.rotation = rotation;
        }
    }

    fn perturb(&mut self, index: usize, seed: usize) -> bool {
        let joint = &mut self.joints[index];
        let (axis, angle) = RESTARTS[seed];
        let axis = match joint.constraint {
            // Further seeds would repeat the hinge's two signed turns.
            JointConstraint::Hinge(hinge) if seed < 2 => hinge.axis,
            JointConstraint::Hinge(_) => return false,
            _ => axis,
        };
        let rotation = joint.rotation;
        let candidate = joint.constraint.apply_to_rotation(
            Quaternion::from_axis_angle(axis.vector(), angle).mul_quat(rotation),
        );
        let components = [
            (candidate.x, rotation.x),
            (candidate.y, rotation.y),
            (candidate.z, rotation.z),
            (candidate.w, rotation.w),
        ];
        // Skip normalization noise without losing tiny changes from zero.
        let matches = |sign: f32| {
            components.iter().all(|&(a, b)| {
                let b = sign * b;
                a == b || (a != 0.0 && b != 0.0 && a.to_bits().abs_diff(b.to_bits()) <= 4)
            })
        };
        // q and -q encode the same rotation.
        if matches(1.0) || matches(-1.0) {
            return false;
        }
        joint.rotation = candidate;
        true
    }
}
