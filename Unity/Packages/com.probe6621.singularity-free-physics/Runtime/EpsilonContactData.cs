namespace SingularityFreePhysics
{
    public struct EpsilonContactData
    {
        public EpsilonBody BodyA;
        public EpsilonBody BodyB;
        public Vector3d ContactPoint;
        public Vector3d Normal;
        public double PenetrationDepth;
        public double RelativeNormalVelocity;
        public double NormalImpulse;
        public double TangentialImpulse;
        public double EnergyDissipated;
    }

    public interface IEpsilonCollisionReceiver
    {
        void OnEpsilonCollision(in EpsilonContactData contact);
    }
}
