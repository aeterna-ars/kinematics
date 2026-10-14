use core::f32::math::*;
use core::intrinsics::*;

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

    #[inline(always)]
    fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    #[inline(always)]
    fn scale(self, factor: f32) -> Self {
        Self { x: self.x * factor, y: self.y * factor }
    }

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self { x: self.x + other.x, y: self.y + other.y }
    }

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        Self { x: self.x - other.x, y: self.y - other.y }
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
    const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    #[inline(always)]
    fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline(always)]
    fn scale(self, factor: f32) -> Self {
        Self { x: self.x * factor, y: self.y * factor, z: self.z * factor }
    }

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        Self { x: self.x - other.x, y: self.y - other.y, z: self.z - other.z }
    }
}

impl Rotatable for Vector3 {
    type Rotation = Quaternion;
    const FORWARD: Self = Self { x: 1.0, y: 0.0, z: 0.0 };

    #[inline(always)]
    fn rotate(self, rot: Self::Rotation) -> Self {
        rot.rotate_vector(self)
    }
}

impl Vector3 {
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
    pub const IDENTITY: Self = Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };

    #[inline(always)]
    pub fn from_axis_angle(axis: Vector3, angle: f32) -> Self {
        if axis.length_squared() < f32::EPSILON { 
            return Self::IDENTITY; 
        }
        let norm_axis = axis.normalize();
        
        let sin_half = sin(angle * 0.5);
        let cos_half = cos(angle * 0.5);

        Self {
            x: norm_axis.x * sin_half,
            y: norm_axis.y * sin_half,
            z: norm_axis.z * sin_half,
            w: cos_half,
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
    pub fn rotate_vector(self, v: Vector3) -> Vector3 {
        let q_vec = Vector3 { x: self.x, y: self.y, z: self.z };
        let cross1 = q_vec.cross(v);
        let cross2 = q_vec.cross(cross1);
        Vector3 {
            x: v.x + 2.0 * (self.w * cross1.x + cross2.x),
            y: v.y + 2.0 * (self.w * cross1.y + cross2.y),
            z: v.z + 2.0 * (self.w * cross1.z + cross2.z),
        }
    }
}
