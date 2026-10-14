use crate::constraint::*;
use crate::space::*;

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

impl<const JOINTS: usize, V: Rotatable> ArmState<JOINTS, V> {
    pub fn solve_fk(&self) -> Vector3 {
        // в пизду нахуй
    }

    pub fn solve_ik(&mut self, max_iters: usize, tolerance: f32, target: Vector3) {
        // андрюха спасай
    }
}