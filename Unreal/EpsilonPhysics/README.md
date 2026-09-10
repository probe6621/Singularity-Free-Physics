# EpsilonPhysics Unreal starter package

This Unreal portion is a self-contained Unreal Engine 5 Runtime plugin with a double-precision, world-independent solver core.

## Included files

- `EpsilonPhysics.uplugin`
- `Source/EpsilonPhysics/EpsilonPhysics.Build.cs`
- `Source/EpsilonPhysics/Private/EpsilonPhysicsModule.cpp`
- `Source/EpsilonPhysics/Public/EpsilonPhysicsSolver.h`
- `Source/EpsilonPhysics/Private/EpsilonPhysicsSolver.cpp`
- `Shaders/EpsilonFieldSolver.ush`
- `Source/EpsilonPhysics/Public/EpsilonNiagaraBridge.h`
- `Source/EpsilonPhysics/Private/EpsilonNiagaraBridge.cpp`

## Design notes

- `FEpsilonParticle` uses `FVector3d` for `Position`, `Velocity`, and `Acceleration`.
- `UEpsilonPhysicsSubsystem` is an `UEngineSubsystem`, so the library core does not touch `UWorld`, actors, components, or physics scene objects.
- Pair potential:
  - `V = -alphaEff MiMj / sqrt(r^2 + eps^2) + beta / (r^2 + eps^2)^2`
  - `F_ij = [alphaEff MiMj / d^(3/2) - 4 beta / d^3] r_ij`, with `r_ij = x_j - x_i` and `d = r^2 + eps^2`
  - particle `i` uses `+F_ij / m_i`, particle `j` uses `-F_ij / m_j`
- `StepSimulation(UPARAM(ref) TArray<FEpsilonParticle>& InOutParticles, double DeltaSeconds)` is the direct API. It recomputes the current acceleration before the first half kick every call, so default or stale incoming accelerations are safe.
- `InitializeSimulation(...)` remains available for callers who want validated configuration plus a managed copy with immediately populated accelerations.
- Invalid inputs fail through `false`, `GetLastError()`, and `LogEpsilonPhysics`. The solver rejects non-finite values, non-positive masses, `beta < 0`, and `epsilon <= 0`.
- `DeltaSeconds <= 0` is a no-op.

## Install

Copy `Unreal/EpsilonPhysics` into your Unreal project's `Plugins/` directory and enable the plugin.

## Direct stepping example

```cpp
UEpsilonPhysicsSubsystem* Solver = GEngine->GetEngineSubsystem<UEpsilonPhysicsSubsystem>();
check(Solver != nullptr);

check(Solver->ConfigureSimulation(2.0, 0.0, 1.0));

TArray<FEpsilonParticle> Particles;

FEpsilonParticle A;
A.Position = FVector3d(0.0, 0.0, 0.0);
A.Mass = 2.0;
Particles.Add(A);

FEpsilonParticle B;
B.Position = FVector3d(1.0, 0.0, 0.0);
B.Mass = 3.0;
Particles.Add(B);

check(Solver->StepSimulation(Particles, 1.0 / 60.0));
check(FMath::IsNearlyEqual(Particles[0].Acceleration.X,  2.1213203435596424, 1.0e-12));
check(FMath::IsNearlyEqual(Particles[1].Acceleration.X, -1.4142135623730951, 1.0e-12));
```

## Initialization sanity check

```cpp
TArray<FEpsilonParticle> Particles;

FEpsilonParticle A;
A.Position = FVector3d(0.0, 0.0, 0.0);
A.Mass = 2.0;
Particles.Add(A);

FEpsilonParticle B;
B.Position = FVector3d(1.0, 0.0, 0.0);
B.Mass = 3.0;
Particles.Add(B);

check(Solver->InitializeSimulation(Particles, 2.0, 0.0, 1.0));

const TArray<FEpsilonParticle>& State = Solver->GetParticlesView();
check(FMath::IsNearlyEqual(State[0].Acceleration.X,  2.1213203435596424, 1.0e-12));
check(FMath::IsNearlyEqual(State[1].Acceleration.X, -1.4142135623730951, 1.0e-12));
```

## Test harness guidance

No AutomationTest source is included because this repository still lacks Unreal test-target boilerplate. Once your project has a runtime test module:

1. call `ConfigureSimulation(...)`,
2. create two particles with zero velocity and arbitrary incoming acceleration values,
3. call direct `StepSimulation(...)`,
4. assert all resulting vectors remain finite,
5. call `InitializeSimulation(...)` and assert the analytical accelerations above,
6. assert rejection of zero/negative mass, `NaN`, `beta < 0`, and `epsilon <= 0`.

## Niagara GPU multi-attractor path

The GPU path is designed for large particle clouds interacting with up to 64 dynamic major attractors (`N x M`, not self-gravity). It is not a true all-pairs N-body solver; 100,000 particles with four attractors is 400,000 field evaluations, while 100,000-body self-gravity would be impractical as an `N x N` loop.

1. Copy this plugin to the project `Plugins/` folder and enable it. The module registers `/Plugin/EpsilonPhysics` as a shader include root.
2. Set the Niagara emitter simulation target to **GPU Compute Sim** and use fixed bounds appropriate to the effect.
3. Create User parameters named `AlphaEff` (float), `Epsilon` (float), `Beta` (float), and `AttractorArray` (Vector4 Array). Each array item packs position XYZ and non-negative mass W.
4. In a Particle Update custom HLSL module, include `/Plugin/EpsilonPhysics/EpsilonFieldSolver.ush` and call `EvaluateEpsilonField_GPU`. Feed `Particles.Acceleration` as `PreviousAcceleration`, use a per-particle boolean initialized false at spawn for `HasPreviousAcceleration`, then set that boolean true after the first update.
5. Write `OutPosition`, `OutVelocity`, and `OutAcceleration` to `Particles.Position`, `Particles.Velocity`, and `Particles.Acceleration`. Remove Niagara's default gravity and drag modules for this path.

The helper performs a genuine kick-drift-kick Velocity-Verlet step: it uses the previous acceleration for the first half kick, evaluates the regularized field at the drifted position, then performs the final half kick. On the first update it derives the initial acceleration from the current position.

Attach `UEpsilonNiagaraBridge` to an actor, assign the target `UNiagaraComponent`, and call `SetAttractors` when attractors change. The bridge validates input, rejects arrays over 64 entries, reports invalid input with `LogEpsilonPhysics`, and writes the User parameters every component tick.
