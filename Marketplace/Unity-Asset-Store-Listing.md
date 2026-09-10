# Unity Asset Store listing draft: Epsilon Physics Pro

## Product title

**Epsilon Physics Pro: Singularity-Free N-Body & Celestial Gravity Engine (DOTS / Burst)**

## Short description

> Singularity-free celestial dynamics for Unity with regularized forces, fixed-step Velocity-Verlet integration, Burst/DOTS acceleration, collision callbacks, and editor visualization tools in the separately licensed Pro package.

Character count: 227 (excluding the quote marker).

## Category and price

- Suggested category: Physics / Physics Engines / Space Simulations
- Suggested launch price: USD $49-$69 under the Unity Asset Store license selected at publication

## Feature copy

- **Regularized central force:** A positive `epsilon` floor regularizes the pair metric so coincident positions do not divide by zero.
- **Fixed-step Velocity-Verlet:** Conservative field integration uses kick-drift-kick stepping. For a time-independent potential and a fixed time step, this is symplectic and bounds long-term energy error; it does not guarantee exact energy conservation for arbitrary time steps.
- **Burst/DOTS compute path:** The Pro package is intended to include `IJobParallelFor`-based, native-array simulation and optional instanced rendering. Exact all-pairs dynamics remains O(N^2); state the particle count only after profiling the shipping package on the named target hardware.
- **Volumetric tension term:** The optional `beta` term produces short-range repulsion. Publish an inversion-radius formula and parameter units only after they are documented and tested for the shipping build.
- **Advanced contacts:** Pro is intended to provide configurable restitution, Coulomb friction, inverse-mass projection, energy reporting, and `IEpsilonCollisionReceiver` callbacks for gameplay integrations.
- **Editor tooling:** Pro may include orbit previews, inversion-shell gizmos, and velocity-vector visualizations. Include only tools actually shipped in the reviewed package.
- **Double-precision simulation state:** The Pro package uses `Vector3d` simulation values. Rendering and Transform synchronization remain subject to Unity's engine coordinate precision.

## Compatibility

State only engine versions and render pipelines tested against the submitted package. Proposed compatibility targets are Unity 2022.3 LTS, later supported LTS releases, URP, HDRP, and Built-in Render Pipeline. Burst platform support is package- and Unity-version dependent; validate every claimed desktop, mobile, console, and CI target before publication.

## Required media

1. Hero: an actual Pro-package particle-disk capture, labeled with the tested Unity version and hardware.
2. Inversion field: an actual scene-view capture of the shipped gizmo and parameter values.
3. Orbit predictor: an editor capture showing the shipped prediction tool.
4. Profiler: a repeatable profiler capture including body count, fixed step, CPU model, Burst version, and render workload.
5. Contacts: an Inspector and gameplay capture of the shipped advanced collision callback.

## Pre-publication checklist

- [ ] Verify every listed Pro feature is present in the distributable package.
- [ ] Run Unity Test Framework tests and manual smoke tests on every claimed Unity version.
- [ ] Measure performance on named hardware; do not publish an unrepeatable frame-time claim.
- [ ] Confirm all third-party package licenses and Asset Store submission requirements.
- [ ] Replace the Community Core Pro-upgrade placeholder with the approved Asset Store URL.
