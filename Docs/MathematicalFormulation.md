# Mathematical formulation

For bodies `i` and `j`, let `r = r_i - r_j`, `d = r.r + epsilon^2`, and require `epsilon > 0`. The pair potential is

```text
V(r) = -alphaEff * M_i * M_j / sqrt(d) + beta / d^2
```

with `beta >= 0`. Its gradient with respect to the first body's position is

```text
grad_i V = [alphaEff * M_i * M_j / d^(3/2) - 4 * beta / d^3] * r
```

The physical pair forces and accelerations are:

```text
F_i = -grad_i V
F_j = +grad_i V
a_i = F_i / M_i
a_j = F_j / M_j
```

Because `d >= epsilon^2`, every denominator is positive. At exactly coincident positions, `r = 0`, so the finite gradient and force are zero.

## Force inversion radius

For positive `alphaEff`, positive masses, and `beta > 0`, the radial force changes direction where the scalar in the potential gradient is zero:

```text
alphaEff * M_i * M_j / d^(3/2) = 4 * beta / d^3
d = (4 * beta / (alphaEff * M_i * M_j))^(2/3)
r_inv = sqrt(d - epsilon^2)
```

A real nonzero `r_inv` exists only when `d >= epsilon^2`. Otherwise the field has no positive-radius inversion point. This expression follows directly from the implemented potential; it should not be replaced with a squared ratio of `beta`, `epsilon`, and mass.

## Integration

At a constant time step `dt`, the package uses kick-drift-kick Velocity-Verlet:

```text
v(t + dt/2) = v(t) + a(t) * dt/2
x(t + dt)   = x(t) + v(t + dt/2) * dt
a(t + dt)   = acceleration(x(t + dt))
v(t + dt)   = v(t + dt/2) + a(t + dt) * dt/2
```

Accelerations are evaluated before the first kick after bodies are added or parameters change. This avoids the common first-step error of beginning with a zero acceleration field.

## Community Core elastic contacts

Point-force regularization prevents a field singularity but does not itself prevent visual overlap. For bodies with radii `R_i` and `R_j`, the Community Core runs a simple discrete contact pass after the drift and before the second field evaluation whenever `|r_i-r_j| <= R_i+R_j`.

For normal `n` from body `j` to body `i`, an approaching pair (`(v_i-v_j).n < 0`) receives a normal impulse

```text
J_n = -2 * ((v_i - v_j).n) / (1/M_i + 1/M_j)
```

and an inverse-mass weighted positional projection. This is a perfectly elastic normal impulse with no friction or collision callback system. It conserves linear momentum and normal relative kinetic energy. Epsilon Physics Pro adds configurable inelastic materials, Coulomb friction, callbacks, and advanced contact controls.

## Units and tuning

The equations are dimensionally consistent only when all chosen simulation units are used consistently. `alphaEff` carries the appropriate gravitational-like units, `epsilon` has distance units, and `beta` has potential-times-distance-to-the-fourth units. Select `dt` small enough to resolve the fastest expected close approach. Reducing `epsilon` or increasing `beta` generally requires a smaller fixed step.
