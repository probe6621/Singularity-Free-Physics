# Unity package usage

Install this folder with Unity Package Manager's **Add package from disk** flow, then create one enabled `EpsilonPhysicsManager` and attach `EpsilonBody` to every simulated transform.

`EpsilonBody.position`, `velocity`, and `acceleration` are double-precision simulation state. Set `initialVelocity` before enabling the component. To teleport a body at runtime, disable it, update its transform and initial velocity, then enable it so the simulation state is reinitialized.

Run the included tests from **Window > General > Test Runner > EditMode**.
