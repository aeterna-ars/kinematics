# Getting started

Requires Rust 1.85 or newer. The library works without `std` or heap
allocation and uses `libm` for trigonometry.

## Example

Two unit-length links with hinges in the XY plane:

```rust
use core::f32::consts::PI;
use kinematics::{
    ArmState, HingeConstraint, Joint, JointConstraint, MotorAxis,
    Quaternion, Vector3,
};

let joint = Joint {
    rotation: Quaternion::IDENTITY,
    length: 1.0,
    base_rotation: Quaternion::IDENTITY,
    constraint: JointConstraint::Hinge(HingeConstraint {
        axis: MotorAxis::Z,
        min_rad: -PI,
        max_rad: PI,
    }),
};
let mut arm = ArmState {
    joints: [joint; 2],
    base_position: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
    effector_position: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
};

let target = Vector3 { x: 1.0, y: 1.0, z: 0.0 };
let report = arm.solve_ik(128, 1e-4, target).expect("valid arm");
assert!(report.converged);
assert!(report.distance <= 1e-4);
assert_eq!(arm.effector_position, arm.solve_fk());
```
