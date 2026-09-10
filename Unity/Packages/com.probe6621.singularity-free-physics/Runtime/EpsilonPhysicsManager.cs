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
                valid &= body != null && body.HasValidMass && body.position.IsFinite() && body.velocity.IsFinite();
            }

            if (!valid && !invalidConfigurationReported)
            {
                Debug.LogError("Epsilon Physics requires finite state, epsilon > 0, beta >= 0, timeScale >= 0, and positive body masses.", this);
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

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
