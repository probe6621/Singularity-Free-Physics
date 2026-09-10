using NUnit.Framework;

namespace SingularityFreePhysics.Tests
{
    public class EpsilonPhysicsMathTests
    {
        [Test]
        public void CoincidentBodies_HaveFinitePotentialAndZeroForce()
        {
            Vector3d gradient = EpsilonPhysicsMath.PotentialGradient(2.0, 3.0, 1.0, 0.5, 0.05, Vector3d.zero);
            double potential = EpsilonPhysicsMath.Potential(2.0, 3.0, 1.0, 0.5, 0.05, Vector3d.zero);

            Assert.That(gradient.SqrMagnitude(), Is.Zero);
            Assert.That(double.IsInfinity(potential) || double.IsNaN(potential), Is.False);
        }

        [Test]
        public void PotentialGradient_MatchesCentralDifference()
        {
            const double h = 1e-5;
            Vector3d separation = new Vector3d(1.2, -0.4, 0.8);
            Vector3d analytical = EpsilonPhysicsMath.PotentialGradient(2.0, 3.0, 1.5, 0.7, 0.02, separation);
            double plus = EpsilonPhysicsMath.Potential(2.0, 3.0, 1.5, 0.7, 0.02, separation + new Vector3d(h, 0.0, 0.0));
            double minus = EpsilonPhysicsMath.Potential(2.0, 3.0, 1.5, 0.7, 0.02, separation - new Vector3d(h, 0.0, 0.0));

            Assert.That(analytical.x, Is.EqualTo((plus - minus) / (2.0 * h)).Within(1e-7));
        }

        [Test]
        public void InvalidEpsilon_IsRejected()
        {
            Assert.Throws<System.ArgumentOutOfRangeException>(() =>
                EpsilonPhysicsMath.PotentialGradient(1.0, 1.0, 1.0, 0.0, 0.0, Vector3d.zero));
        }
    }
}
