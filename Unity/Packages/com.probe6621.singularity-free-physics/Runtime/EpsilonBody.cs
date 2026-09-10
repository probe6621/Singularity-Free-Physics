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

        [Header("Physical Boundary & Collision")]
        [Tooltip("Non-negative physical radius in simulation distance units.")]
        [Min(0.0f)]
        public double radius = 1.0;

        [Range(0.0f, 1.0f)]
        [Tooltip("Impact restitution: zero is inelastic and one is elastic.")]
        public double restitution = 0.8;

        [Min(0.0f)]
        [Tooltip("Non-negative Coulomb friction coefficient.")]
        public double friction = 0.2;

        [HideInInspector] public Vector3d position;
        [HideInInspector] public Vector3d velocity;
        [HideInInspector] public Vector3d acceleration;

        internal bool HasValidMass => !double.IsNaN(mass) && !double.IsInfinity(mass) && mass > 0.0;
        internal bool HasValidCollisionProperties =>
            !double.IsNaN(radius) && !double.IsInfinity(radius) && radius >= 0.0 &&
            !double.IsNaN(restitution) && !double.IsInfinity(restitution) && restitution >= 0.0 && restitution <= 1.0 &&
            !double.IsNaN(friction) && !double.IsInfinity(friction) && friction >= 0.0;

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
            restitution = IsFinite(restitution) ? System.Math.Min(System.Math.Max(restitution, 0.0), 1.0) : 0.8;
            friction = IsFinite(friction) ? System.Math.Max(friction, 0.0) : 0.0;
        }

        public void SyncTransform()
        {
            transform.position = position.ToVector3();
        }

        internal void NotifyCollision(in EpsilonContactData contact)
        {
            MonoBehaviour[] behaviours = GetComponents<MonoBehaviour>();
            for (int i = 0; i < behaviours.Length; i++)
            {
                if (behaviours[i] is IEpsilonCollisionReceiver receiver)
                {
                    receiver.OnEpsilonCollision(in contact);
                }
            }
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
