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

        [HideInInspector] public Vector3d position;
        [HideInInspector] public Vector3d velocity;
        [HideInInspector] public Vector3d acceleration;

        internal bool HasValidMass => !double.IsNaN(mass) && !double.IsInfinity(mass) && mass > 0.0;

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
        }

        public void SyncTransform()
        {
            transform.position = position.ToVector3();
        }
    }
}
