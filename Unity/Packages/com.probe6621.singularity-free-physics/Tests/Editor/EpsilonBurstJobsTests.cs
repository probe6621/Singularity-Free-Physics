using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace SingularityFreePhysics.Tests
{
    public class EpsilonBurstJobsTests
    {
        [Test]
        public void PairwiseJob_CoincidentBodiesProduceFiniteZeroAcceleration()
        {
            NativeArray<double3> positions = new NativeArray<double3>(2, Allocator.TempJob);
            NativeArray<double> masses = new NativeArray<double>(2, Allocator.TempJob);
            NativeArray<double3> accelerations = new NativeArray<double3>(2, Allocator.TempJob);
            try
            {
                masses[0] = 2.0;
                masses[1] = 3.0;
                new EpsilonPairwiseAccelerationJob
                {
                    Positions = positions,
                    Masses = masses,
                    Accelerations = accelerations,
                    AlphaEff = 1.0,
                    EpsilonSquared = 0.25,
                    Beta = 0.05
                }.Schedule(2, 1).Complete();

                Assert.That(math.all(accelerations[0] == double3.zero), Is.True);
                Assert.That(math.all(accelerations[1] == double3.zero), Is.True);
            }
            finally
            {
                positions.Dispose();
                masses.Dispose();
                accelerations.Dispose();
            }
        }

        [Test]
        public void PairwiseJob_PositiveAlphaAttractsBodies()
        {
            NativeArray<double3> positions = new NativeArray<double3>(2, Allocator.TempJob);
            NativeArray<double> masses = new NativeArray<double>(2, Allocator.TempJob);
            NativeArray<double3> accelerations = new NativeArray<double3>(2, Allocator.TempJob);
            try
            {
                positions[1] = new double3(1.0, 0.0, 0.0);
                masses[0] = 2.0;
                masses[1] = 3.0;
                new EpsilonPairwiseAccelerationJob
                {
                    Positions = positions,
                    Masses = masses,
                    Accelerations = accelerations,
                    AlphaEff = 2.0,
                    EpsilonSquared = 1.0,
                    Beta = 0.0
                }.Schedule(2, 1).Complete();

                Assert.That(accelerations[0].x, Is.EqualTo(2.1213203435596424).Within(1e-12));
                Assert.That(accelerations[1].x, Is.EqualTo(-1.4142135623730951).Within(1e-12));
            }
            finally
            {
                positions.Dispose();
                masses.Dispose();
                accelerations.Dispose();
            }
        }
    }
}
