using System;

namespace SingularityFreePhysics
{
    /// <summary>Pure field functions shared by simulation code and tests.</summary>
    public static class EpsilonPhysicsMath
    {
        public static double Potential(double massA, double massB, double alphaEff, double epsilon, double beta, Vector3d separation)
        {
            ValidateInputs(massA, massB, alphaEff, epsilon, beta, separation);
            double denominator = separation.SqrMagnitude() + epsilon * epsilon;
            return -(alphaEff * massA * massB) / Math.Sqrt(denominator) + beta / (denominator * denominator);
        }

        /// <summary>
        /// Returns grad(V) with respect to the first body's position. The physical force is its negative.
        /// </summary>
        public static Vector3d PotentialGradient(
            double massA,
            double massB,
            double alphaEff,
            double epsilon,
            double beta,
            Vector3d separation)
        {
            ValidateInputs(massA, massB, alphaEff, epsilon, beta, separation);
            double denominator = separation.SqrMagnitude() + epsilon * epsilon;
            double denominatorThreeHalves = denominator * Math.Sqrt(denominator);
            double denominatorCubed = denominator * denominator * denominator;
            double attraction = alphaEff * massA * massB / denominatorThreeHalves;
            double repulsion = 4.0 * beta / denominatorCubed;
            return separation * (attraction - repulsion);
        }

        private static void ValidateInputs(
            double massA,
            double massB,
            double alphaEff,
            double epsilon,
            double beta,
            Vector3d separation)
        {
            if (!IsFinite(massA) || !IsFinite(massB) || massA <= 0.0 || massB <= 0.0 ||
                !IsFinite(alphaEff) || !IsFinite(epsilon) || epsilon <= 0.0 ||
                !IsFinite(beta) || beta < 0.0 || !separation.IsFinite())
            {
                throw new ArgumentOutOfRangeException(nameof(epsilon), "Inputs must be finite; masses and epsilon must be positive and beta non-negative.");
            }
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
