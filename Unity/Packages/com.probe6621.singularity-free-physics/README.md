# Unity package usage

Install this folder with Unity Package Manager's **Add package from disk** flow, then create one enabled `EpsilonPhysicsManager` and attach `EpsilonBody` to every simulated transform.

`EpsilonBody.position`, `velocity`, and `acceleration` are double-precision simulation state. Set `initialVelocity` before enabling the component. To teleport a body at runtime, disable it, update its transform and initial velocity, then enable it so the simulation state is reinitialized.

Run the included tests from **Window > General > Test Runner > EditMode**.

## Basic elastic contacts

Set a non-negative `radius` on each `EpsilonBody`; collision handling is enabled by default on `EpsilonPhysicsManager`. After the Verlet drift, overlapping bodies are projected apart by inverse mass and approaching bodies receive a perfectly elastic, momentum-conserving normal impulse. Exact coincident centers use a deterministic fallback normal to avoid a division by zero.

For Burst/DOTS high-capacity simulation, inelastic collision response, contact callbacks, visualizers, and production demos, see **Epsilon Physics Pro**.
