#[derive(Clone, Copy, PartialEq)]
pub enum MotorAxis { X, Y, Z }

#[derive(Clone, Copy)]
pub enum JointConstraint {
    None,
    Hinge(HingeConstraint),
    Ball(BallConstraint),
}

impl JointConstraint {
    #[inline(always)]
    pub fn apply_to_angle(&self, angle: f32) -> f32 {
        match self {
            Self::None => angle,
            Self::Hinge(c) => c.apply(angle),
            Self::Ball(c) => c.apply(angle),
        }
    }
}

#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
pub struct BallConstraint {
    pub max_cone_rad: f32,
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