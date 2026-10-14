use core::f32::consts::PI;
use libm::{atan2f, cosf as cos, hypotf, sinf as sin, sqrtf as sqrt};

pub trait VectorSpace: Sized + Copy + PartialEq {
    const ZERO: Self;

    fn dot(self, other: Self) -> f32;
    fn scale(self, factor: f32) -> Self;
    fn add(self, other: Self) -> Self;
    fn sub(self, other: Self) -> Self;

    #[inline(always)]
    fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline(always)]
    fn length(self) -> f32 {
        sqrt(self.length_squared())
    }

    #[inline(always)]
    fn normalize(self) -> Self {
        let len_sq = self.length_squared();
        if len_sq > f32::EPSILON {
            self.scale(1.0 / sqrt(len_sq))
        } else {
            Self::ZERO
        }
    }
}

pub trait Rotatable: Sized + VectorSpace {
    type Rotation: Copy + PartialEq + Default;
    const FORWARD: Self;
    fn rotate(self, rot: Self::Rotation) -> Self;
}

#[derive(PartialEq, Clone, Copy, Default, Debug)]
#[repr(C)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

impl VectorSpace for Vector2 {
    const ZERO: Self = Self { x: 0.0, y: 0.0 };

    fn length(self) -> f32 {
        hypotf(self.x, self.y)
    }

    fn normalize(self) -> Self {
        let scale = self.x.abs().max(self.y.abs());
        if scale == 0.0 || !scale.is_finite() {
            return Self::ZERO;
        }
        let scaled = Self {
            x: self.x / scale,
            y: self.y / scale,
        };
        scaled.scale(1.0 / sqrt(scaled.length_squared()))
    }

    #[inline(always)]
    fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    #[inline(always)]
    fn scale(self, factor: f32) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Rotatable for Vector2 {
    type Rotation = f32;
    const FORWARD: Self = Self { x: 1.0, y: 0.0 };

    #[inline(always)]
    fn rotate(self, angle: Self::Rotation) -> Self {
        let sin_a = sin(angle);
        let cos_a = cos(angle);
        Self {
            x: self.x * cos_a - self.y * sin_a,
            y: self.x * sin_a + self.y * cos_a,
        }
    }
}

#[derive(PartialEq, Clone, Copy, Default, Debug)]
#[repr(C)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl VectorSpace for Vector3 {
    const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    fn length(self) -> f32 {
        hypotf(hypotf(self.x, self.y), self.z)
    }

    fn normalize(self) -> Self {
        let scale = self.x.abs().max(self.y.abs()).max(self.z.abs());
        if scale == 0.0 || !scale.is_finite() {
            return Self::ZERO;
        }
        let scaled = Self {
            x: self.x / scale,
            y: self.y / scale,
            z: self.z / scale,
        };
        scaled.scale(1.0 / sqrt(scaled.length_squared()))
    }

    #[inline(always)]
    fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline(always)]
    fn scale(self, factor: f32) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Rotatable for Vector3 {
    type Rotation = Quaternion;
    const FORWARD: Self = Self {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline(always)]
    fn rotate(self, rot: Self::Rotation) -> Self {
        rot.rotate_vector(self)
    }
}

impl Vector3 {
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    #[inline(always)]
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
#[repr(C, align(16))]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quaternion {
    #[inline(always)]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    #[inline(always)]
    pub fn from_axis_angle(axis: Vector3, angle: f32) -> Self {
        let norm_axis = axis.normalize();
        if norm_axis == Vector3::ZERO {
            return Self::IDENTITY;
        }

        let sin_half = sin(angle * 0.5);
        let cos_half = cos(angle * 0.5);

        Self {
            x: norm_axis.x * sin_half,
            y: norm_axis.y * sin_half,
            z: norm_axis.z * sin_half,
            w: cos_half,
        }
    }

    /// Normalize a rotation; the zero quaternion maps to identity.
    pub fn normalized(self) -> Self {
        let scale = self
            .x
            .abs()
            .max(self.y.abs())
            .max(self.z.abs())
            .max(self.w.abs());
        if scale == 0.0 || !scale.is_finite() {
            return Self::IDENTITY;
        }
        // Scale before squaring to avoid overflow and underflow.
        let q = Self {
            x: self.x / scale,
            y: self.y / scale,
            z: self.z / scale,
            w: self.w / scale,
        };
        let inverse = 1.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        Self {
            x: q.x * inverse,
            y: q.y * inverse,
            z: q.z * inverse,
            w: q.w * inverse,
        }
    }

    /// Quaternion conjugate; the inverse for unit quaternions.
    pub fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite() && self.w.is_finite()
    }

    pub(crate) fn is_valid(self) -> bool {
        self.is_finite() && (self.x != 0.0 || self.y != 0.0 || self.z != 0.0 || self.w != 0.0)
    }

    /// Shortest rotation between directions; zero inputs produce identity.
    /// Opposite directions use a deterministic perpendicular axis.
    pub fn from_rotation_arc(from: Vector3, to: Vector3) -> Self {
        let from = from.normalize();
        let to = to.normalize();
        if from == Vector3::ZERO || to == Vector3::ZERO {
            return Self::IDENTITY;
        }
        let axis = from.cross(to);
        let sine = axis.length();
        let cosine = from.dot(to).clamp(-1.0, 1.0);
        if sine > 0.0 {
            // atan2 retains tiny angles when the dot product rounds to one.
            Self::from_axis_angle(axis, atan2f(sine, cosine))
        } else if cosine < 0.0 {
            let reference = if from.x.abs() < from.y.abs() {
                Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                }
            } else {
                Vector3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                }
            };
            Self::from_axis_angle(from.cross(reference), PI)
        } else {
            Self::IDENTITY
        }
    }

    #[inline(always)]
    pub fn mul_quat(self, r: Self) -> Self {
        Self {
            w: self.w * r.w - self.x * r.x - self.y * r.y - self.z * r.z,
            x: self.w * r.x + self.x * r.w + self.y * r.z - self.z * r.y,
            y: self.w * r.y - self.x * r.z + self.y * r.w + self.z * r.x,
            z: self.w * r.z + self.x * r.y - self.y * r.x + self.z * r.w,
        }
    }

    #[inline(always)]
    /// Rotate a vector. `self` must be a unit quaternion.
    pub fn rotate_vector(self, v: Vector3) -> Vector3 {
        let q_vec = Vector3 {
            x: self.x,
            y: self.y,
            z: self.z,
        };
        let cross1 = q_vec.cross(v);
        let cross2 = q_vec.cross(cross1);
        Vector3 {
            x: v.x + 2.0 * (self.w * cross1.x + cross2.x),
            y: v.y + 2.0 * (self.w * cross1.y + cross2.y),
            z: v.z + 2.0 * (self.w * cross1.z + cross2.z),
        }
    }
}
