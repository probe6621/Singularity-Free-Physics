using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace SingularityFreePhysics
{
    [DefaultExecutionOrder(-90)]
    public sealed class EpsilonBurstManager : MonoBehaviour
    {
        private const double MinimumEpsilon = 1e-9;

        [Header("Physical Field Coupling")]
        public double alphaEff = 1.0;
        [Min(0.000001f)] public double epsilon = 0.5;
        [Min(0.0f)] public double beta = 0.05;

        [Header("Simulation Timing")]
        [Min(0.0f)] public double timeScale = 1.0;

        [Header("Job Tuning")]
        [Min(1)] public int batchSize = 64;

        private NativeArray<double3> positions;
        private NativeArray<double3> velocities;
        private NativeArray<double3> accelerations;
        private NativeArray<double> masses;
        private JobHandle currentJobHandle;
        private bool hasScheduledWork;
        private bool isInitialized;
        private double initializedAlphaEff;
        private double initializedEpsilon;
        private double initializedBeta;

        public int ParticleCount { get; private set; }
        public bool IsInitialized => isInitialized;

        public void Initialize(double3[] initialPositions, double3[] initialVelocities, double[] initialMasses)
        {
            if (initialPositions == null || initialVelocities == null || initialMasses == null)
            {
                throw new ArgumentNullException(nameof(initialPositions), "All initial state arrays are required.");
            }
            if (initialPositions.Length != initialVelocities.Length || initialPositions.Length != initialMasses.Length)
            {
                throw new ArgumentException("Initial position, velocity, and mass arrays must have equal lengths.");
            }

            ValidateConfiguration();
            ValidateInitialState(initialPositions, initialVelocities, initialMasses);
            DisposeSimulation();

            ParticleCount = initialPositions.Length;
            positions = new NativeArray<double3>(ParticleCount, Allocator.Persistent);
            velocities = new NativeArray<double3>(ParticleCount, Allocator.Persistent);
            accelerations = new NativeArray<double3>(ParticleCount, Allocator.Persistent);
            masses = new NativeArray<double>(ParticleCount, Allocator.Persistent);
            positions.CopyFrom(initialPositions);
            velocities.CopyFrom(initialVelocities);
            masses.CopyFrom(initialMasses);

            RecomputeAccelerations();
            isInitialized = true;
        }

        public void CompletePendingSimulation()
        {
            if (!hasScheduledWork)
            {
                return;
            }

            currentJobHandle.Complete();
            hasScheduledWork = false;
        }

        public NativeArray<double3> GetPositions()
        {
            RequireInitialized();
            CompletePendingSimulation();
            return positions;
        }

        public NativeArray<double3> GetVelocities()
        {
            RequireInitialized();
            CompletePendingSimulation();
            return velocities;
        }

        public NativeArray<double> GetMasses()
        {
            RequireInitialized();
            CompletePendingSimulation();
            return masses;
        }

        private void FixedUpdate()
        {
            if (!isInitialized)
            {
                return;
            }

            CompletePendingSimulation();
            ValidateConfiguration();
            if (FieldParametersChanged())
            {
                RecomputeAccelerations();
            }

            double deltaTime = Time.fixedDeltaTime * timeScale;
            if (!IsFinite(deltaTime) || deltaTime <= 0.0)
            {
                return;
            }

            JobHandle kickDrift = new EpsilonKickDriftJob
            {
                Positions = positions,
                Velocities = velocities,
                Accelerations = accelerations,
                DeltaTime = deltaTime
            }.Schedule(ParticleCount, batchSize);

            JobHandle calculateAcceleration = ScheduleAccelerationCalculation(kickDrift);
            currentJobHandle = new EpsilonFinalKickJob
            {
                Velocities = velocities,
                Accelerations = accelerations,
                DeltaTime = deltaTime
            }.Schedule(ParticleCount, batchSize, calculateAcceleration);
            hasScheduledWork = true;
        }

        private void LateUpdate()
        {
            CompletePendingSimulation();
        }

        private void OnDisable()
        {
            CompletePendingSimulation();
        }

        private void OnDestroy()
        {
            DisposeSimulation();
        }

        private void OnValidate()
        {
            epsilon = IsFinite(epsilon) ? Math.Max(epsilon, MinimumEpsilon) : 0.5;
            beta = IsFinite(beta) ? Math.Max(beta, 0.0) : 0.0;
            timeScale = IsFinite(timeScale) ? Math.Max(timeScale, 0.0) : 1.0;
            batchSize = Math.Max(batchSize, 1);
        }

        private JobHandle ScheduleAccelerationCalculation(JobHandle dependency)
        {
            return new EpsilonPairwiseAccelerationJob
            {
                Positions = positions,
                Masses = masses,
                Accelerations = accelerations,
                AlphaEff = alphaEff,
                EpsilonSquared = epsilon * epsilon,
                Beta = beta
            }.Schedule(ParticleCount, batchSize, dependency);
        }

        private void RecomputeAccelerations()
        {
            ScheduleAccelerationCalculation(default).Complete();
            initializedAlphaEff = alphaEff;
            initializedEpsilon = epsilon;
            initializedBeta = beta;
        }

        private bool FieldParametersChanged() =>
            alphaEff != initializedAlphaEff || epsilon != initializedEpsilon || beta != initializedBeta;

        private void DisposeSimulation()
        {
            CompletePendingSimulation();
            if (positions.IsCreated) positions.Dispose();
            if (velocities.IsCreated) velocities.Dispose();
            if (accelerations.IsCreated) accelerations.Dispose();
            if (masses.IsCreated) masses.Dispose();
            ParticleCount = 0;
            isInitialized = false;
            initializedAlphaEff = 0.0;
            initializedEpsilon = 0.0;
            initializedBeta = 0.0;
        }

        private void ValidateConfiguration()
        {
            if (!IsFinite(alphaEff) || !IsFinite(epsilon) || epsilon < MinimumEpsilon ||
                !IsFinite(beta) || beta < 0.0 || !IsFinite(timeScale) || timeScale < 0.0)
            {
                throw new InvalidOperationException("Epsilon Burst Physics requires finite alphaEff, epsilon > 0, beta >= 0, and timeScale >= 0.");
            }
        }

        private static void ValidateInitialState(double3[] initialPositions, double3[] initialVelocities, double[] initialMasses)
        {
            for (int i = 0; i < initialMasses.Length; i++)
            {
                if (!IsFinite(initialMasses[i]) || initialMasses[i] <= 0.0 ||
                    !IsFinite(initialPositions[i]) || !IsFinite(initialVelocities[i]))
                {
                    throw new ArgumentException($"Particle {i} must have finite position, velocity, and a strictly positive finite mass.");
                }
            }
        }

        private void RequireInitialized()
        {
            if (!isInitialized)
            {
                throw new InvalidOperationException("Initialize must be called before accessing simulation buffers.");
            }
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        private static bool IsFinite(double3 value) => math.all(math.isfinite(value));
    }
}
