use crate::{Quaternion, Rotatable, Vector3, VectorSpace};
use core::f32::consts::{PI, TAU};
use libm::atan2f;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum MotorAxis {
    X,
    Y,
    Z,
}

impl MotorAxis {
    pub const fn vector(self) -> Vector3 {
        match self {
            Self::X => Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Self::Y => Vector3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Self::Z => Vector3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum JointConstraint {
    None,
    Hinge(HingeConstraint),
    Ball(BallConstraint),
}

impl JointConstraint {
    /// Project a local rotation relative to [`crate::Joint::base_rotation`].
    /// Requires valid limits; see the crate docs for accepted ranges.
    pub fn apply_to_rotation(&self, rotation: Quaternion) -> Quaternion {
        let rotation = rotation.normalized();
        match self {
            Self::None => rotation,
            Self::Hinge(c) => {
                let v = Vector3 {
                    x: rotation.x,
                    y: rotation.y,
                    z: rotation.z,
                };
                // Canonicalize q/-q before atan2 to preserve tiny angles.
                let mut component = v.dot(c.axis.vector());
                if rotation.w == 0.0 {
                    component = component.abs();
                } else if rotation.w < 0.0 {
                    component = -component;
                }
                let angle = 2.0 * atan2f(component, rotation.w.abs());
                // Compare both wraps to choose the nearest limit at +/-pi.
                let alternate = if angle < 0.0 {
                    angle + TAU
                } else {
                    angle - TAU
                };
                let first = c.apply(angle);
                let second = c.apply(alternate);
                let limited = if wrap_angle(angle - second).abs() < wrap_angle(angle - first).abs()
                {
                    second
                } else {
                    first
                };
                Quaternion::from_axis_angle(c.axis.vector(), limited)
            }
            Self::Ball(c) => {
                let direction = rotation.rotate_vector(Vector3::FORWARD);
                let swing_axis = Vector3::FORWARD.cross(direction);
                let angle = atan2f(swing_axis.length(), direction.x);
                if angle <= c.max_cone_rad {
                    return rotation;
                }
                let axis = if swing_axis != Vector3::ZERO {
                    swing_axis
                } else {
                    // Opposite directions have no unique swing axis.
                    MotorAxis::Z.vector()
                };
                let twist = Quaternion {
                    x: rotation.x,
                    y: 0.0,
                    z: 0.0,
                    w: rotation.w,
                }
                .normalized();
                Quaternion::from_axis_angle(axis, c.max_cone_rad)
                    .mul_quat(twist)
                    .normalized()
            }
        }
    }

    pub(crate) fn is_valid(&self) -> bool {
        match self {
            Self::None => true,
            Self::Hinge(c) => {
                c.min_rad.is_finite()
                    && c.max_rad.is_finite()
                    && c.min_rad >= -PI
                    && c.max_rad <= PI
                    && c.min_rad <= c.max_rad
            }
            Self::Ball(c) => c.max_cone_rad.is_finite() && (0.0..=PI).contains(&c.max_cone_rad),
        }
    }

    #[inline(always)]
    pub fn apply_to_angle(&self, angle: f32) -> f32 {
        match self {
            Self::None => angle,
            Self::Hinge(c) => c.apply(angle),
            Self::Ball(c) => c.apply(angle),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HingeConstraint {
    pub axis: MotorAxis,
    pub min_rad: f32,
    pub max_rad: f32,
}

impl HingeConstraint {
    #[inline(always)]
    pub fn apply(&self, angle: f32) -> f32 {
        if angle < self.min_rad {
            self.min_rad
        } else if angle > self.max_rad {
            self.max_rad
        } else {
            angle
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BallConstraint {
    pub max_cone_rad: f32,
}

fn wrap_angle(angle: f32) -> f32 {
    let angle = angle % TAU;
    if angle > PI {
        angle - TAU
    } else if angle < -PI {
        angle + TAU
    } else {
        angle
    }
}

impl BallConstraint {
    #[inline(always)]
    pub fn apply(&self, angle: f32) -> f32 {
        if angle < 0.0 {
            0.0
        } else if angle > self.max_cone_rad {
            self.max_cone_rad
        } else {
            angle
        }
    }
}
