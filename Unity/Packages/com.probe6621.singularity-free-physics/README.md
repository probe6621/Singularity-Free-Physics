# Unity package usage

Install this folder with Unity Package Manager's **Add package from disk** flow, then create one enabled `EpsilonPhysicsManager` and attach `EpsilonBody` to every simulated transform.

`EpsilonBody.position`, `velocity`, and `acceleration` are double-precision simulation state. Set `initialVelocity` before enabling the component. To teleport a body at runtime, disable it, update its transform and initial velocity, then enable it so the simulation state is reinitialized.

Run the included tests from **Window > General > Test Runner > EditMode**.

## Discrete collisions

Set a non-negative `radius` on each `EpsilonBody`; collision handling is enabled by default on `EpsilonPhysicsManager`. After the Verlet drift, overlapping bodies are projected apart by inverse mass and approaching bodies receive a momentum-conserving normal impulse plus a Coulomb friction impulse. `restitution` must be within `[0, 1]`; `friction` must be non-negative.

Attach a component implementing `IEpsilonCollisionReceiver` to either body to receive `OnEpsilonCollision(in EpsilonContactData contact)`. The contact normal points from `BodyB` to `BodyA`; the same payload is sent to both receivers. Contacts are also reported for overlapping bodies that are already separating, with zero impulse, so gameplay can observe persistent contact. Exact coincident centers use a deterministic fallback normal to avoid a division by zero.

## Burst high-capacity path

`EpsilonBurstManager` owns native Structure-of-Arrays buffers and schedules Burst-compiled `IJobParallelFor` jobs for Kick-Drift-Kick Velocity-Verlet integration. Call `Initialize(double3[] positions, double3[] velocities, double[] masses)` once; all arrays must have the same length, finite values, and strictly positive masses. Call `GetPositions()` only for rendering or inspection; it completes scheduled work so readers never race jobs.

Import `Samples~/ParticleCloudDemo` through Package Manager for an optional `Graphics.RenderMeshInstanced` stress sample. This package targets Unity 2022.3+ because that sample uses `RenderMeshInstanced`.
