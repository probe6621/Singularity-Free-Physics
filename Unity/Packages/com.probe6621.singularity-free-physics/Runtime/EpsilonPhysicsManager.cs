using System;
using System.Collections.Generic;
using UnityEngine;

namespace SingularityFreePhysics
{
    [DefaultExecutionOrder(-100)]
    public sealed class EpsilonPhysicsManager : MonoBehaviour
    {
        private const double MinimumEpsilon = 1e-9;
        private static readonly List<EpsilonBody> Bodies = new List<EpsilonBody>();

        public static EpsilonPhysicsManager Instance { get; private set; }

        [Header("Field Parameters")]
        [Tooltip("Effective attraction coupling (G - K * eta^2).")]
        public double alphaEff = 1.0;

        [Tooltip("Positive irreducible core scale. It prevents singularities at coincident positions.")]
        [Min(0.000001f)]
        public double epsilon = 0.5;

        [Tooltip("Non-negative short-range volumetric energy tension coupling.")]
        [Min(0.0f)]
        public double beta = 0.05;

        [Header("Simulation Timing")]
        [Tooltip("Positive multiplier applied to Unity's fixed time step.")]
        [Min(0.0f)]
        public double timeScale = 1.0;

        [Header("Discrete Contacts")]
        [Tooltip("Applies rigid-body impulses and positional projection after the Verlet drift.")]
        public bool collisionsEnabled = true;

        private bool accelerationIsCurrent;
        private bool invalidConfigurationReported;

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Debug.LogError("Only one EpsilonPhysicsManager may be active.", this);
                enabled = false;
                return;
            }

            Instance = this;
            accelerationIsCurrent = false;
        }

        private void OnDestroy()
        {
            if (Instance == this)
            {
                Instance = null;
            }
        }

        private void OnValidate()
        {
            epsilon = IsFinite(epsilon) ? Math.Max(epsilon, MinimumEpsilon) : 0.5;
            beta = IsFinite(beta) ? Math.Max(beta, 0.0) : 0.0;
            timeScale = IsFinite(timeScale) ? Math.Max(timeScale, 0.0) : 1.0;
            accelerationIsCurrent = false;
        }

        public static void RegisterBody(EpsilonBody body)
        {
            if (body == null || Bodies.Contains(body))
            {
                return;
            }

            Bodies.Add(body);
            if (Instance != null)
            {
                Instance.accelerationIsCurrent = false;
            }
        }

        public static void UnregisterBody(EpsilonBody body)
        {
            if (body != null && Bodies.Remove(body) && Instance != null)
            {
                Instance.accelerationIsCurrent = false;
            }
        }

        private void FixedUpdate()
        {
            RemoveDestroyedBodies();
            if (!ValidateConfiguration())
            {
                return;
            }

            double dt = Time.fixedDeltaTime * timeScale;
            if (!IsFinite(dt) || dt <= 0.0 || Bodies.Count == 0)
            {
                return;
            }

            if (!accelerationIsCurrent)
            {
                ComputeAccelerations();
            }

            // Velocity-Verlet: kick, drift, recompute the conservative field, then kick.
            for (int i = 0; i < Bodies.Count; i++)
            {
                EpsilonBody body = Bodies[i];
                body.velocity += body.acceleration * (0.5 * dt);
                body.position += body.velocity * dt;
            }

            if (collisionsEnabled)
            {
                ResolveCollisions();
            }

            ComputeAccelerations();

            for (int i = 0; i < Bodies.Count; i++)
            {
                EpsilonBody body = Bodies[i];
                body.velocity += body.acceleration * (0.5 * dt);
                body.SyncTransform();
            }
        }

        private void ComputeAccelerations()
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                Bodies[i].acceleration = Vector3d.zero;
            }

            for (int i = 0; i < Bodies.Count; i++)
            {
                EpsilonBody first = Bodies[i];
                for (int j = i + 1; j < Bodies.Count; j++)
                {
                    EpsilonBody second = Bodies[j];
                    Vector3d separation = first.position - second.position;

                    // This is grad(V); applying -grad(V) produces the physical acceleration.
                    Vector3d potentialGradient = EpsilonPhysicsMath.PotentialGradient(
                        first.mass, second.mass, alphaEff, epsilon, beta, separation);
                    first.acceleration -= potentialGradient / first.mass;
                    second.acceleration += potentialGradient / second.mass;
                }
            }

            accelerationIsCurrent = true;
        }

        private bool ValidateConfiguration()
        {
            bool valid = IsFinite(alphaEff) && IsFinite(epsilon) && epsilon >= MinimumEpsilon &&
                         IsFinite(beta) && beta >= 0.0 && IsFinite(timeScale) && timeScale >= 0.0;
            for (int i = 0; i < Bodies.Count; i++)
            {
                EpsilonBody body = Bodies[i];
                valid &= body != null && body.HasValidMass && body.HasValidCollisionProperties &&
                    body.position.IsFinite() && body.velocity.IsFinite();
            }

            if (!valid && !invalidConfigurationReported)
            {
                Debug.LogError("Epsilon Physics requires finite state, epsilon > 0, beta >= 0, timeScale >= 0, positive body masses, and valid collision properties.", this);
                invalidConfigurationReported = true;
            }
            else if (valid)
            {
                invalidConfigurationReported = false;
            }

            return valid;
        }

        private void RemoveDestroyedBodies()
        {
            if (Bodies.RemoveAll(body => body == null) > 0)
            {
                accelerationIsCurrent = false;
            }
        }

        private void ResolveCollisions()
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                EpsilonBody first = Bodies[i];
                for (int j = i + 1; j < Bodies.Count; j++)
                {
                    EpsilonBody second = Bodies[j];
                    double combinedRadius = first.radius + second.radius;
                    if (combinedRadius <= 0.0)
                    {
                        continue;
                    }

                    Vector3d separation = first.position - second.position;
                    double distanceSquared = separation.SqrMagnitude();
                    if (distanceSquared > combinedRadius * combinedRadius)
                    {
                        continue;
                    }

                    Vector3d normal;
                    double distance;
                    if (distanceSquared > 1e-24)
                    {
                        distance = Math.Sqrt(distanceSquared);
                        normal = separation / distance;
                    }
                    else
                    {
                        distance = 0.0;
                        normal = first.GetInstanceID() < second.GetInstanceID()
                            ? new Vector3d(1.0, 0.0, 0.0)
                            : new Vector3d(-1.0, 0.0, 0.0);
                    }

                    double inverseMassFirst = 1.0 / first.mass;
                    double inverseMassSecond = 1.0 / second.mass;
                    double inverseMassSum = inverseMassFirst + inverseMassSecond;
                    double penetration = combinedRadius - distance;
                    Vector3d correction = normal * (penetration / inverseMassSum);
                    first.position += correction * inverseMassFirst;
                    second.position -= correction * inverseMassSecond;

                    Vector3d relativeVelocityBefore = first.velocity - second.velocity;
                    double normalVelocity = Vector3d.Dot(relativeVelocityBefore, normal);
                    double normalImpulseMagnitude = 0.0;
                    double tangentialImpulseMagnitude = 0.0;
                    if (normalVelocity < 0.0)
                    {
                        double restitution = Math.Sqrt(first.restitution * second.restitution);
                        normalImpulseMagnitude = -(1.0 + restitution) * normalVelocity / inverseMassSum;
                        Vector3d impulse = normal * normalImpulseMagnitude;

                        Vector3d tangentVelocity = relativeVelocityBefore - normal * normalVelocity;
                        double tangentSpeedSquared = tangentVelocity.SqrMagnitude();
                        if (tangentSpeedSquared > 1e-24)
                        {
                            double tangentSpeed = Math.Sqrt(tangentSpeedSquared);
                            double friction = Math.Sqrt(first.friction * second.friction);
                            tangentialImpulseMagnitude = -Math.Min(
                                friction * normalImpulseMagnitude,
                                tangentSpeed / inverseMassSum);
                            impulse += (tangentVelocity / tangentSpeed) * tangentialImpulseMagnitude;
                        }

                        first.velocity += impulse * inverseMassFirst;
                        second.velocity -= impulse * inverseMassSecond;
                    }

                    Vector3d relativeVelocityAfter = first.velocity - second.velocity;
                    double reducedMass = 1.0 / inverseMassSum;
                    double energyDissipated = Math.Max(
                        0.0,
                        0.5 * reducedMass * (relativeVelocityBefore.SqrMagnitude() - relativeVelocityAfter.SqrMagnitude()));
                    EpsilonContactData contact = new EpsilonContactData
                    {
                        BodyA = first,
                        BodyB = second,
                        ContactPoint = second.position + normal * second.radius,
                        Normal = normal,
                        PenetrationDepth = penetration,
                        RelativeNormalVelocity = normalVelocity,
                        NormalImpulse = normalImpulseMagnitude,
                        TangentialImpulse = tangentialImpulseMagnitude,
                        EnergyDissipated = energyDissipated
                    };
                    first.NotifyCollision(in contact);
                    second.NotifyCollision(in contact);
                }
            }
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
