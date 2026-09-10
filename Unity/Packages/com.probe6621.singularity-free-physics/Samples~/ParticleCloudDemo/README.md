# Particle Cloud Demo

Import this sample with Package Manager, add an `EpsilonBurstManager` and `EpsilonParticleCloudDemo` to the same empty scene object, then assign an instancing-enabled material and a mesh.

The sample uses the independent-particle `IJobParallelFor` acceleration calculation. Its O(N^2) force evaluation deliberately favors exact all-pair dynamics over a tree approximation, so actual capacity depends on the target CPU, Burst settings, fixed time step, and other game workload. Profile on the intended platform; 10,000 particles means roughly 100 million directed interactions per simulation step.

The demo's initial circular speeds approximate the softened attraction from the central mass. Mutual particle interactions, vertical offsets, and the optional short-range tension term mean the disk is a stress sample, not an analytically exact equilibrium.
