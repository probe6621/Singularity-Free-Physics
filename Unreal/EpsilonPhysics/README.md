# EpsilonPhysics Unreal starter package

This Unreal portion is a self-contained Unreal Engine 5 Runtime plugin with a double-precision, world-independent solver core.

## Included files

- `EpsilonPhysics.uplugin`
- `Source/EpsilonPhysics/EpsilonPhysics.Build.cs`
- `Source/EpsilonPhysics/Private/EpsilonPhysicsModule.cpp`
- `Source/EpsilonPhysics/Public/EpsilonPhysicsSolver.h`
- `Source/EpsilonPhysics/Public/EpsilonContactTypes.h`
- `Source/EpsilonPhysics/Private/EpsilonPhysicsSolver.cpp`
- `Shaders/EpsilonFieldSolver.ush`
- `Source/EpsilonPhysics/Public/EpsilonNiagaraBridge.h`
- `Source/EpsilonPhysics/Private/EpsilonNiagaraBridge.cpp`

## Design notes

- `FEpsilonParticle` uses `FVector3d` for `Position`, `Velocity`, and `Acceleration`.
- `FEpsilonParticle` also carries sphere collision material data: `Radius >= 0`, `Restitution in [0, 1]`, and `Friction >= 0`. The defaults (`Radius = 0`) preserve the old field-only behavior.
- `UEpsilonPhysicsSubsystem` is an `UEngineSubsystem`, so the library core does not touch `UWorld`, actors, components, or physics scene objects.
- Pair potential:
  - `V = -alphaEff MiMj / sqrt(r^2 + eps^2) + beta / (r^2 + eps^2)^2`
  - `F_ij = [alphaEff MiMj / d^(3/2) - 4 beta / d^3] r_ij`, with `r_ij = x_j - x_i` and `d = r^2 + eps^2`
  - particle `i` uses `+F_ij / m_i`, particle `j` uses `-F_ij / m_j`
- `ConfigureCollision(...)` toggles collision handling and validates positional projection settings. The default collision configuration is enabled with full overlap projection and zero slop.
- `StepSimulation(UPARAM(ref) TArray<FEpsilonParticle>& InOutParticles, double DeltaSeconds)` is the direct API. It recomputes the current acceleration before the first half kick every call, performs the first half kick and drift, resolves sphere collisions, then evaluates the field at the post-collision positions for the second half kick.
- `InitializeSimulation(...)` remains available for callers who want validated configuration plus a managed copy with immediately populated accelerations.
- Collision impulses use the B-to-A contact normal `normalize(rA - rB)` when available, fall back deterministically for exact coincidence, apply only for approaching motion (`vn < 0`), combine restitution/friction via geometric mean, and include Coulomb tangent friction.
- Overlap projection always runs for penetrating pairs, even when they are already separating, so overlap cleanup is not skipped.
- `GetLastContacts()` / `GetLastContactsView()` expose the last step's resolved contacts, and `OnCollisionResolved` broadcasts each `FEpsilonContactInfo`.
- `FEpsilonContactInfo::NormalEnergyDissipated` uses reduced mass and the pre/post normal relative speeds.
- Invalid inputs fail through `false`, `GetLastError()`, and `LogEpsilonPhysics`. The solver rejects non-finite values, non-positive masses, negative radius, restitution outside `[0, 1]`, negative friction, `beta < 0`, invalid collision settings, and `epsilon <= 0`.
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

## Collision example

```cpp
check(Solver->ConfigureSimulation(2.0, 0.0, 1.0));
check(Solver->ConfigureCollision(true, 1.0, 0.0));

TArray<FEpsilonParticle> Particles;

FEpsilonParticle A;
A.Position = FVector3d(-25.0, 0.0, 0.0);
A.Velocity = FVector3d(50.0, 0.0, 0.0);
A.Mass = 1.0;
A.Radius = 20.0;
A.Restitution = 0.25;
A.Friction = 0.5;
Particles.Add(A);

FEpsilonParticle B;
B.Position = FVector3d(25.0, 0.0, 0.0);
B.Velocity = FVector3d(-50.0, 0.0, 0.0);
B.Mass = 1.0;
B.Radius = 20.0;
B.Restitution = 0.25;
B.Friction = 0.5;
Particles.Add(B);

check(Solver->StepSimulation(Particles, 0.5));

const TArray<FEpsilonContactInfo>& Contacts = Solver->GetLastContactsView();
check(Contacts.Num() == 1);
check(Contacts[0].bAppliedImpulse);
check(Contacts[0].PenetrationDepth >= 0.0);
check(Contacts[0].NormalEnergyDissipated >= 0.0);
```

## Test harness guidance

No AutomationTest source is included because this repository still lacks Unreal test-target boilerplate and this environment does not include an Unreal compiler/toolchain to add or execute one here. Once your project has a runtime test module:

1. call `ConfigureSimulation(...)`,
2. call `ConfigureCollision(true, 1.0, 0.0)`,
3. create two particles with arbitrary incoming acceleration values and radii large enough to collide during the step,
4. call direct `StepSimulation(...)`,
5. assert all resulting vectors remain finite, the bodies no longer overlap, and `GetLastContacts()` contains the expected pair with `NormalEnergyDissipated >= 0`,
6. bind to `OnCollisionResolved` and assert one callback per resolved contact,
7. call managed `InitializeSimulation(...)` plus `StepSimulation(double DeltaSeconds)` and assert contacts are still reported there,
8. assert rejection of zero/negative mass, negative radius, `NaN`, restitution outside `[0, 1]`, negative friction, `beta < 0`, invalid collision projection settings, and `epsilon <= 0`.

## Niagara GPU multi-attractor path

The GPU path is designed for large particle clouds interacting with up to 64 dynamic major attractors (`N x M`, not self-gravity). It is not a true all-pairs N-body solver; 100,000 particles with four attractors is 400,000 field evaluations, while 100,000-body self-gravity would be impractical as an `N x N` loop.

1. Copy this plugin to the project `Plugins/` folder and enable it. The module registers `/Plugin/EpsilonPhysics` as a shader include root.
2. Set the Niagara emitter simulation target to **GPU Compute Sim** and use fixed bounds appropriate to the effect.
3. Create User parameters named `AlphaEff` (float), `Epsilon` (float), `Beta` (float), and `AttractorArray` (Vector4 Array). Each array item packs position XYZ and non-negative mass W.
4. In a Particle Update custom HLSL module, include `/Plugin/EpsilonPhysics/EpsilonFieldSolver.ush` and call `EvaluateEpsilonField_GPU`. Feed `Particles.Acceleration` as `PreviousAcceleration`, use a per-particle boolean initialized false at spawn for `HasPreviousAcceleration`, then set that boolean true after the first update.
5. Write `OutPosition`, `OutVelocity`, and `OutAcceleration` to `Particles.Position`, `Particles.Velocity`, and `Particles.Acceleration`. Remove Niagara's default gravity and drag modules for this path.

The helper performs a genuine kick-drift-kick Velocity-Verlet step: it uses the previous acceleration for the first half kick, evaluates the regularized field at the drifted position, then performs the final half kick. On the first update it derives the initial acceleration from the current position.

Attach `UEpsilonNiagaraBridge` to an actor, assign the target `UNiagaraComponent`, and call `SetAttractors` when attractors change. The bridge validates input, rejects arrays over 64 entries, reports invalid input with `LogEpsilonPhysics`, and writes the User parameters every component tick.
