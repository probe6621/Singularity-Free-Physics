using UnityEngine;

namespace SingularityFreePhysics
{
    [DisallowMultipleComponent]
    public sealed class EpsilonBody : MonoBehaviour
    {
        [Header("Mass & Kinematics")]
        [Tooltip("Strictly positive mass in simulation units.")]
        [Min(0.000001f)]
        public double mass = 1.0;

        [Tooltip("Velocity applied when this component is enabled.")]
        public Vector3 initialVelocity = Vector3.zero;

        [Header("Physical Boundary")]
        [Tooltip("Non-negative physical radius for the Community Core elastic contact pass.")]
        [Min(0.0f)]
        public double radius = 1.0;

        [HideInInspector] public Vector3d position;
        [HideInInspector] public Vector3d velocity;
        [HideInInspector] public Vector3d acceleration;

        internal bool HasValidMass => !double.IsNaN(mass) && !double.IsInfinity(mass) && mass > 0.0;
        internal bool HasValidRadius => !double.IsNaN(radius) && !double.IsInfinity(radius) && radius >= 0.0;

        private void OnEnable()
        {
            position = new Vector3d(transform.position);
            velocity = new Vector3d(initialVelocity);
            acceleration = Vector3d.zero;
            EpsilonPhysicsManager.RegisterBody(this);
        }

        private void OnDisable()
        {
            EpsilonPhysicsManager.UnregisterBody(this);
        }

        private void OnValidate()
        {
            if (mass <= 0.0 || double.IsNaN(mass) || double.IsInfinity(mass))
            {
                mass = 0.000001;
            }
            radius = IsFinite(radius) ? System.Math.Max(radius, 0.0) : 0.0;
        }

        public void SyncTransform()
        {
            transform.position = position.ToVector3();
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
