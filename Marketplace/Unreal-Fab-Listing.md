# Unreal Fab listing draft: Epsilon Physics Pro

## Product title

**Epsilon Physics Pro: Singularity-Free N-Body & Niagara GPU Gravitational Engine**

## Short description

> Singularity-free celestial dynamics for Unreal Engine with double-precision CPU tools, Niagara GPU multi-attractor effects, collision events, and source code in the separately licensed Pro package.

Character count: 197 (excluding the quote marker).

## Category and price

- Suggested category: Code Plugins / Physics / Particle Systems / Blueprints
- Suggested launch price: USD $59-$79 under the Fab license selected at publication

## Feature copy

- **Regularized central force:** A positive softening length makes the pair-force denominator finite at coincident positions.
- **CPU runtime subsystem:** The Pro package is intended to include validated double-precision `FVector3d` CPU stepping with direct-array and managed-state usage.
- **Niagara GPU multi-attractor module:** The Pro GPU feature is an N x M field path for many visual particles and a bounded number of heavy attractors. It is not a general GPU N x N self-gravity solver. State the supported attractor cap and performance only after profiling the shipping asset.
- **Kick-drift-kick integration:** Niagara HLSL uses a prior acceleration, drifted-position evaluation, and final half kick. Validate the exact Niagara graph, parameter names, and engine versions before shipping.
- **Live attractor bridge:** The proposed `UEpsilonNiagaraBridge` sends validated attractor data and user parameters to a target Niagara component.
- **Advanced contacts and events:** Pro is intended to provide configurable inelastic hard-sphere response, friction, projection controls, contact information, and Blueprint-compatible collision delegates.
- **Large-world positioning:** `FVector3d` supports double-precision simulation state. Origin rebasing, actor integration, and replication are separate product features and must not be advertised unless implemented and tested.

## Compatibility

List only Unreal Engine releases, platforms, renderers, and target types built and tested from the submitted plugin. Proposed targets should be confirmed through clean project builds on every stated UE version. This product does **not** claim built-in network replication; applications must replicate their own authoritative state unless the Pro package explicitly implements and documents replication.

## Required media

1. Hero: a capture from the shipping Pro Niagara system, with particle count, GPU, UE version, and scene workload.
2. Field visualization: the shipping inversion-shell visualizer with actual parameters.
3. Blueprint: the published collision delegate or bridge component in a working graph.
4. GPU profiler: a reproducible RenderDoc, Insights, or GPU profiler capture with attractor count and effect settings.
5. Plugin integration: a clean-project install and enabled-plugin view for the submitted UE release.

## Pre-publication checklist

- [ ] Verify every feature and screenshot against the packaged Pro plugin.
- [ ] Build and test all advertised Unreal Engine versions and platform targets.
- [ ] Confirm Niagara GPU behavior on supported hardware and fallbacks on unsupported targets.
- [ ] Measure and document performance with reproducible scene settings.
- [ ] Verify Fab licensing, code-plugin requirements, and all third-party notices.
- [ ] Replace the Community Core Pro-upgrade placeholder with the approved Fab URL.
