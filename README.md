# kinematics

Forward and inverse kinematics for 3D joint chains, written in Rust.

The library calculates the end position of a chain and adjusts its joints
toward a target. It supports hinge and ball constraints and runs without
`std` or heap allocation.

Status: early development.

## Design

Chains have a fixed number of joints. Each joint stores its length, rest
frame and local rotation.

FK composes these rotations from the base to the tip. IK uses CCD and
keeps the closest pose found within the joint limits.

The solver handles position only. CCD can miss a reachable target.

## Usage

[Getting started](docs/getting-started.md) · [Documentation](docs/README.md)

Requires Rust 1.85 or newer.

## Development

See the [development guide](docs/development.md) for checks, Rust documentation
and embedded targets.
