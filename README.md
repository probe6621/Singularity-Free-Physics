# Singularity-Free Physics Community Core

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An open-source, dual-engine Community Core for regularized central-force simulations. It provides finite pair interactions at coincident positions, fixed-step Velocity-Verlet integration, and simple elastic hard-sphere contacts for Unity and Unreal Engine.

> Need Burst/DOTS high-capacity simulation, Unreal Niagara GPU acceleration, inelastic/friction contacts, callbacks, visualizers, or production demos? **Epsilon Physics Pro** is the commercial upgrade. Marketplace links will be published here.

Internal publication drafts for the separately distributed Pro package are in [Marketplace/](/Users/paulroberts/.copilot/chats/28ab1c47-0475-453d-aeb7-dc9f83edef68/Singularity-Free-Physics/Marketplace).

## Contents

- [`Unity/Packages/com.probe6621.singularity-free-physics`](Unity/Packages/com.probe6621.singularity-free-physics): Unity Package Manager package with the classic MonoBehaviour CPU solver and EditMode math tests.
- [`Unreal/EpsilonPhysics`](Unreal/EpsilonPhysics): Unreal Engine runtime plugin with a double-precision CPU solver.
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
- The Community Core is intended for small body counts; exact pair evaluation is O(N^2). Profile your target hardware and use a modest body count.
- The CPU Unity and Unreal solvers optionally augment the field with discrete radius-based perfectly elastic contacts. Epsilon Physics Pro adds high-throughput compute paths and advanced inelastic contacts.

## Validation

Unity includes EditMode NUnit tests for coincident-body finiteness, the analytical gradient, and invalid-parameter rejection. The Unreal plugin README includes a focused runtime test-harness recipe for the C++ solver.
