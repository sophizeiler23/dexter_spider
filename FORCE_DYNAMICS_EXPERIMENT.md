# Rigidbody Surface-Adhesion Experiment

This clone starts from commit `1d874a4` of the Dexter spider project. It uses
the forward/down probing, artificial surface gravity, and wall-alignment ideas
described by `PhilS94/Unity-Procedural-IK-Wall-Walking-Spider`, but contains an
original implementation and does not copy its copyrighted source or assets.

## What owns movement now

`DexterFrontLegIK` still performs the existing Dexter calibration, gesture
recognition, force-to-speed mapping, tetrapod gait, jumping intent, terrain
targeting, and leg IK. It no longer writes the root transform while
`SpiderRigidbodyDynamics` is enabled.

`SpiderRigidbodyDynamics` creates/configures a non-kinematic `Rigidbody` and:

- probes both toward the current surface and ahead for wall transitions;
- applies world gravity and active spider adhesion as actual forces;
- applies clearance, propulsion, and friction forces through planted foot
  contact points with `Rigidbody.AddForceAtPosition`;
- aligns to changing surfaces using torque and angular damping;
- lets collisions, mass, velocity, gravity, and angular velocity determine the
  final root pose.

The component is installed automatically on the same GameObject as
`DexterFrontLegIK` when Play Mode initializes. Its tuning sections are
**Rigidbody Body**, **Surface-Relative Gravity and Adhesion**, **Dexter Foot
Propulsion**, and **Surface Alignment Torque**.

## Validation

Unity compiled with no C# errors. The built-in eight-second synthetic Dexter
walk completed without runtime exceptions and moved the physical root about
3.62 world units while maintaining a stable visible pose.
