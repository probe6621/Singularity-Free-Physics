# Singularity-Free Physics (Community Core)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Release](https://img.shields.io/badge/Release-v1.0.0--community-blue.svg)](https://github.com/probe6621/Singularity-Free-Physics/tree/v1.0.0-community)

An open-source, double-precision N-body physics foundation for **Unity** and **Unreal Engine 5**. It regularizes the central-force denominator with a positive `epsilon` floor, avoiding division by zero and non-finite pair forces at coincident positions.

> ### Looking for production-scale compute (10k-100k+ bodies)?
> The separately licensed **Epsilon Physics Pro** package is planned to extend this Community Core with:
> - **Unity:** Multi-threaded Burst/DOTS solvers and Scene View orbit trajectory tooling.
> - **Unreal Engine 5:** Niagara GPU Compute HLSL integration and a live dynamic-attractor bridge.
> - **Contact dynamics:** Inelastic collision response, Coulomb friction, and collision callbacks.
>
> Unity Asset Store and Unreal Fab links will be published here when the Pro package is available.

## Feature comparison

| Feature | Community Core (Open Source) | Studio Pro (Commercial) |
| :--- | :---: | :---: |
| Licensing | MIT | Commercial / marketplace license |
| Regularized pair potential | Included | Included |
| Volumetric tension (`beta`) | Included | Included |
| Numerical integrator | CPU Velocity-Verlet | CPU, SIMD, and GPU paths |
| Precision model | `Vector3d` / `FVector3d` simulation state | Planned large-world tooling |
| Compute scale | Small O(N^2) CPU systems | Planned high-throughput solvers |
| Parallel / GPU compute | Not included | Planned Burst/DOTS and Niagara paths |
| Collision handling | Elastic hard-sphere contacts | Planned inelastic response and friction |
| Collision callbacks | Not included | Planned callbacks and delegates |
| Editor visualizers and demos | Not included | Planned production tooling and demos |

## Theoretical foundation

For pair separation `r_ij = r_i - r_j`, define `d = r_ij.r_ij + epsilon^2` with `epsilon > 0`:

```text
V(r_ij) = -alphaEff * M_i * M_j / sqrt(d) + beta / d^2
grad_i V = [alphaEff * M_i * M_j / d^(3/2) - 4 * beta / d^3] * r_ij
F_i = -grad_i V
```

Since `d >= epsilon^2`, the potential and pair force remain finite. Velocity-Verlet is symplectic for a time-independent conservative potential with fixed `dt`; it bounds long-term energy error rather than guaranteeing exact energy conservation for arbitrary parameters or time steps.

For the derivation of the force-inversion radius, integration details, contact model, and units, see [MathematicalFormulation.md](Docs/MathematicalFormulation.md).

## Repository structure

```text
Singularity-Free-Physics/
├── LICENSE
├── Docs/MathematicalFormulation.md
├── Marketplace/                         # Qualified drafts for the separate Pro product
├── Unity/Packages/com.probe6621.singularity-free-physics/
│   ├── Runtime/                          # CPU Velocity-Verlet and elastic contacts
│   └── Tests/Editor/                     # EditMode numerical regression tests
└── Unreal/EpsilonPhysics/
    └── Source/EpsilonPhysics/            # CPU EngineSubsystem and elastic contacts
```

## Quick start

### Unity

1. In Package Manager, add the package from the Git URL:
   `https://github.com/probe6621/Singularity-Free-Physics.git?path=/Unity/Packages/com.probe6621.singularity-free-physics#v1.0.0-community`
2. Add `EpsilonPhysicsManager` to one scene object.
3. Add `EpsilonBody` to every simulated object and configure positive `mass`, non-negative `radius`, and `initialVelocity`.
4. Choose a positive `epsilon`, non-negative `beta`, and a fixed time step appropriate to the fastest expected encounter.

The manager owns the simulated state and synchronizes it to Transform after each fixed step. Do not simultaneously drive these transforms with Rigidbody, animation, or another movement system.

### Unreal Engine 5

1. Copy [Unreal/EpsilonPhysics](Unreal/EpsilonPhysics) into your project's `Plugins/` folder and enable it.
2. Configure and step the engine subsystem:

```cpp
UEpsilonPhysicsSubsystem* Solver = GEngine->GetEngineSubsystem<UEpsilonPhysicsSubsystem>();
if (Solver != nullptr && Solver->ConfigureSimulation(1.0, 0.05, 10.0))
{
    Solver->StepSimulation(MyParticleArray, DeltaSeconds);
}
```

Use `FEpsilonParticle` values with strictly positive `Mass`, non-negative `Radius`, and double-precision `FVector3d` state.

## Community contact handling

The Community Core includes silent hard-sphere contact handling: inverse-mass positional de-penetration and perfectly elastic normal impulses (`e = 1`). Exact coincident centers use a deterministic finite fallback normal. The Pro scope reserves inelastic damping, friction, collision events, and callbacks.

## Validation and license

Unity includes EditMode tests for coincident-body finiteness, the analytical gradient, and invalid-parameter rejection. See the [Unreal plugin README](Unreal/EpsilonPhysics/README.md) for its C++ test-harness guidance.

The Community Core is released under the [MIT License](LICENSE). Contributions that preserve mathematical correctness and documentation accuracy are welcome.
