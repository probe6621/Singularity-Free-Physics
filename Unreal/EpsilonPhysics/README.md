# EpsilonPhysics Unreal Community Core

This Unreal portion is the MIT Community Core plugin: a self-contained Unreal Engine 5 runtime module with a double-precision CPU solver for regularized central-force particle simulation plus simple hard-sphere collisions.

## Included files

- `EpsilonPhysics.uplugin`
- `Source/EpsilonPhysics/EpsilonPhysics.Build.cs`
- `Source/EpsilonPhysics/Private/EpsilonPhysicsModule.cpp`
- `Source/EpsilonPhysics/Public/EpsilonPhysicsSolver.h`
- `Source/EpsilonPhysics/Private/EpsilonPhysicsSolver.cpp`

## Design notes

- `FEpsilonParticle` stores `Position`, `Velocity`, `Acceleration`, `Mass`, and `Radius`.
- `Radius` must remain finite and non-negative. `Mass` must remain finite and strictly positive.
- `UEpsilonPhysicsSubsystem` is an `UEngineSubsystem`, so the solver core does not depend on `UWorld`, actors, components, or physics scene objects.
- Pair potential:
  - `V = -alphaEff MiMj / sqrt(r^2 + eps^2) + beta / (r^2 + eps^2)^2`
  - `F_ij = [alphaEff MiMj / d^(3/2) - 4 beta / d^3] r_ij`, with `r_ij = x_j - x_i` and `d = r^2 + eps^2`
  - particle `i` uses `+F_ij / m_i`, particle `j` uses `-F_ij / m_j`
- `StepSimulation(UPARAM(ref) TArray<FEpsilonParticle>& InOutParticles, double DeltaSeconds)` is the direct API. Each step recomputes the current acceleration before the first half kick, performs kick-drift-kick integration, resolves radius-based hard-sphere collisions after drift with full inverse-mass positional projection, then evaluates the field at the post-collision positions for the final half kick.
- Collision response uses a perfectly elastic normal impulse (`e = 1`) only for approaching pairs. Exact coincident centers fall back deterministically to the relative-velocity direction or a fixed axis.
- Invalid inputs fail through `false`, `GetLastError()`, and `LogEpsilonPhysics`. The solver rejects non-finite values, non-positive masses, negative radius, `beta < 0`, and `epsilon <= 0`.
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

## Collision example

```cpp
check(Solver->ConfigureSimulation(0.0, 0.0, 1.0));

TArray<FEpsilonParticle> Particles;

FEpsilonParticle A;
A.Position = FVector3d(-25.0, 0.0, 0.0);
A.Velocity = FVector3d(50.0, 0.0, 0.0);
A.Mass = 1.0;
A.Radius = 20.0;
Particles.Add(A);

FEpsilonParticle B;
B.Position = FVector3d(25.0, 0.0, 0.0);
B.Velocity = FVector3d(-50.0, 0.0, 0.0);
B.Mass = 1.0;
B.Radius = 20.0;
Particles.Add(B);

check(Solver->StepSimulation(Particles, 0.5));
check(FMath::IsNearlyEqual(Particles[0].Position.X, -20.0, 1.0e-12));
check(FMath::IsNearlyEqual(Particles[1].Position.X,  20.0, 1.0e-12));
check(FMath::IsNearlyEqual(Particles[0].Velocity.X, -50.0, 1.0e-12));
check(FMath::IsNearlyEqual(Particles[1].Velocity.X,  50.0, 1.0e-12));
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

No AutomationTest source is included because this repository still lacks Unreal test-target boilerplate and this environment does not include an Unreal compiler/toolchain to add or execute one here. Once your project has a runtime test module:

1. call `ConfigureSimulation(...)`,
2. step a small particle array through the direct `StepSimulation(...)` API,
3. assert all resulting vectors remain finite,
4. add an overlap case and assert the bodies no longer overlap after the step,
5. assert a head-on equal-mass collision swaps the normal velocity components as expected,
6. call managed `InitializeSimulation(...)` plus `StepSimulation(double DeltaSeconds)` and assert the managed state stays finite,
7. assert rejection of zero/negative mass, negative radius, `NaN`, `beta < 0`, and `epsilon <= 0`.
