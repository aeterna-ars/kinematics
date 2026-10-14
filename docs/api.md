# Solver API

## Forward kinematics

`solve_fk(&self) -> Vector3` evaluates the current pose. It neither changes the
arm nor clamps joint limits, and it does not read or update `effector_position`.
FK expects finite positions, nonnegative finite lengths and finite, nonzero
quaternions; it does not validate inputs or report arithmetic overflow.

## Inverse kinematics

`solve_ik(max_iters, tolerance, target) -> Result<IkReport, IkError>` uses
constrained cyclic coordinate descent (CCD), visiting joints from the tip to
the base. It projects the initial pose into its limits, retains the best pose
found, and updates both joint rotations and `effector_position`.

## Result and iteration budget

`IkReport` contains `converged`, the number of completed `iterations`, and the
final `distance` to the target. The iteration count never exceeds `max_iters`.
A zero budget still normalizes rotations, enforces limits, and refreshes the
effector cache. Zero-length links and empty chains are supported.

## Convergence

Unreachable targets, local minima, and exhausted budgets return a report with
`converged == false`, not an error. The solver tries up to six deterministic
pose perturbations when progress stalls, including for collinear chains.
It visits joints from base to tip, staggering seed directions across joints
to spread the limited attempts across both joints and axes. Unchanged rotations
are skipped. Hinges use only the two directions of their own axis.
CCD is a local method: a reachable target can still fail to converge, and the
reported distance is not proof that a target is unreachable. Reusing a previous
solution as the starting pose is useful for tracking a moving target.

## Errors

Invalid inputs return `IkError` without changing any arm state. This includes
nonfinite positions, negative or nonfinite tolerance/lengths, zero or nonfinite
quaternions, invalid limits, and arithmetic overflow. The old effector cache is
ignored. Floating-point precision still limits useful tolerances, especially
when a short chain is translated far from the origin.

## Cost and scope

The implementation uses O(N) stack space and O(N²) work per sweep for N joints.
It solves end-effector **position**, without collision avoidance or an
end-effector orientation target. Solvers are provided for `ArmState<N, Vector3>`;
`Vector2` remains available as a math utility.
