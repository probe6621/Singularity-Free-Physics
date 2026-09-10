# Singularity-Free Physics

A dual-engine, double-precision N-body starter package for regularized central-force simulations. It provides finite pair interactions at coincident positions and fixed-step Velocity-Verlet integration for Unity and Unreal Engine.

## Contents

- [`Unity/Packages/com.probe6621.singularity-free-physics`](Unity/Packages/com.probe6621.singularity-free-physics): Unity Package Manager package, including classic MonoBehaviour and Burst/Job System solvers plus EditMode math tests.
- [`Unreal/EpsilonPhysics`](Unreal/EpsilonPhysics): Unreal Engine runtime module with a double-precision CPU solver plus a Niagara GPU multi-attractor field path.
- [`Docs/Mathematical-Model.md`](Docs/Mathematical-Model.md): formulation, units, and integration notes.

## Unity quick start

1. Add the package from disk in the Unity Package Manager, selecting `Unity/Packages/com.probe6621.singularity-free-physics`.
2. Create one scene object with `EpsilonPhysicsManager`.
3. Add `EpsilonBody` to each simulated object and set a strictly positive `mass` and `initialVelocity`.
4. Set `epsilon` to a meaningful positive length in your simulation units. Tune `alphaEff`, `beta`, and Unity's fixed time step together.

The manager owns simulated positions and writes them to each body's transform after each fixed step. Do not also drive these transforms with `Rigidbody`, animation, or another movement script.

## Unreal quick start

Copy `Unreal/EpsilonPhysics` into your project's `Plugins/` directory, enable the plugin, call `ConfigureSimulation`, then call `StepSimulation` on an `UEpsilonPhysicsSubsystem` instance with an array of `FEpsilonParticle` values. See the Unreal README for module setup and ownership guidance.

## Production notes

- All masses must be finite and strictly positive; `epsilon` must be finite and positive; `beta` must be finite and non-negative.
- The solver intentionally refuses invalid state rather than allowing a NaN to contaminate the simulation.
- Velocity-Verlet is symplectic for a time-independent conservative potential at a constant step. It bounds long-term energy error; it does **not** guarantee exact energy conservation or stability for every choice of step and parameters.
- Pair evaluation is O(N^2). Use a Barnes-Hut/FMM approximation, GPU implementation, or spatial partitioning for very large body counts.
- The optional Burst path uses SIMD-friendly native arrays and multithreaded per-particle reductions. Benchmark on the target hardware; exact 10,000-body all-pairs simulation requires roughly 100 million directed interactions per fixed step.
- The Niagara GPU path supports up to 64 live attractors for large `N x M` particle effects; it is not an `N x N` self-gravity solver. See the plugin README for the required Niagara parameter and scratch-module setup.
- The CPU Unity and Unreal solvers optionally augment the field with discrete radius-based impulses for elastic/inelastic contacts. This preserves linear momentum while intentionally allowing collision energy dissipation; see the engine-specific READMEs.

## Validation

Unity includes EditMode NUnit tests for coincident-body finiteness, the analytical gradient, invalid-parameter rejection, and Burst pair-force direction. The Unreal plugin README includes a focused runtime test-harness recipe for the C++ solver and Niagara bridge.
