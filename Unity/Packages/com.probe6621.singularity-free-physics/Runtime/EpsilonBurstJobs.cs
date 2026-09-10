using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace SingularityFreePhysics
{
    [BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Standard)]
    public struct EpsilonKickDriftJob : IJobParallelFor
    {
        public NativeArray<double3> Positions;
        public NativeArray<double3> Velocities;
        [ReadOnly] public NativeArray<double3> Accelerations;
        [ReadOnly] public double DeltaTime;

        public void Execute(int index)
        {
            double3 halfStepVelocity = Velocities[index] + Accelerations[index] * (0.5 * DeltaTime);
            Positions[index] += halfStepVelocity * DeltaTime;
            Velocities[index] = halfStepVelocity;
        }
    }

    [BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Standard)]
    public struct EpsilonPairwiseAccelerationJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double3> Positions;
        [ReadOnly] public NativeArray<double> Masses;
        [WriteOnly] public NativeArray<double3> Accelerations;
        [ReadOnly] public double AlphaEff;
        [ReadOnly] public double EpsilonSquared;
        [ReadOnly] public double Beta;

        public void Execute(int index)
        {
            double mass = Masses[index];
            double inverseMass = 1.0 / mass;
            double3 position = Positions[index];
            double3 totalAcceleration = double3.zero;

            for (int otherIndex = 0; otherIndex < Positions.Length; otherIndex++)
            {
                if (otherIndex == index)
                {
                    continue;
                }

                // r points from this body to the other; positive alpha therefore attracts.
                double3 separation = Positions[otherIndex] - position;
                double distanceTerm = math.lengthsq(separation) + EpsilonSquared;
                double distanceThreeHalves = distanceTerm * math.sqrt(distanceTerm);
                double distanceCubed = distanceTerm * distanceTerm * distanceTerm;
                double gradientScale = (AlphaEff * mass * Masses[otherIndex] / distanceThreeHalves)
                    - (4.0 * Beta / distanceCubed);

                totalAcceleration += separation * (gradientScale * inverseMass);
            }

            Accelerations[index] = totalAcceleration;
        }
    }

    [BurstCompile(FloatMode = FloatMode.Fast, FloatPrecision = FloatPrecision.Standard)]
    public struct EpsilonFinalKickJob : IJobParallelFor
    {
        public NativeArray<double3> Velocities;
        [ReadOnly] public NativeArray<double3> Accelerations;
        [ReadOnly] public double DeltaTime;

        public void Execute(int index)
        {
            Velocities[index] += Accelerations[index] * (0.5 * DeltaTime);
        }
    }
}
