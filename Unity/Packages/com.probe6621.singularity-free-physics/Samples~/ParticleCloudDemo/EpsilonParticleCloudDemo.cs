using SingularityFreePhysics;
using Unity.Mathematics;
using UnityEngine;

namespace SingularityFreePhysics.Samples
{
    /// <summary>Optional 10,000-body visual stress sample using GPU instancing.</summary>
    public sealed class EpsilonParticleCloudDemo : MonoBehaviour
    {
        private const int MaxInstancesPerDraw = 1023;

        [Header("Demo Configuration")]
        [Min(2)] public int totalParticles = 10000;
        [Min(0.000001f)] public double centralMass = 5000.0;
        [Min(0.000001f)] public double diskRadius = 150.0;
        public Mesh particleMesh;
        public Material particleMaterial;

        [Header("Engine Reference")]
        public EpsilonBurstManager burstManager;

        private Matrix4x4[][] renderBatches;
        private MaterialPropertyBlock propertyBlock;

        private void Start()
        {
            if (burstManager == null)
            {
                burstManager = GetComponent<EpsilonBurstManager>();
            }
            if (burstManager == null || particleMesh == null || particleMaterial == null)
            {
                Debug.LogError("Particle cloud demo requires an EpsilonBurstManager, mesh, and material.", this);
                enabled = false;
                return;
            }

            propertyBlock = new MaterialPropertyBlock();
            SpawnSwarm();
        }

        private void Update()
        {
            if (!burstManager.IsInitialized)
            {
                return;
            }

            NativeArrayToRenderBatches(burstManager.GetPositions());
            for (int batch = 0; batch < renderBatches.Length; batch++)
            {
                Graphics.RenderMeshInstanced(
                    new RenderParams(particleMaterial) { matProps = propertyBlock },
                    particleMesh,
                    0,
                    renderBatches[batch],
                    renderBatches[batch].Length);
            }
        }

        private void SpawnSwarm()
        {
            int count = Mathf.Max(totalParticles, 2);
            double3[] initialPositions = new double3[count];
            double3[] initialVelocities = new double3[count];
            double[] initialMasses = new double[count];
            Random random = new Random(42);

            initialMasses[0] = centralMass;
            for (int index = 1; index < count; index++)
            {
                double radius = random.NextDouble(5.0, math.max(5.001, diskRadius));
                double angle = random.NextDouble(0.0, math.PI * 2.0);
                initialPositions[index] = new double3(
                    radius * math.cos(angle),
                    random.NextDouble(-1.5, 1.5),
                    radius * math.sin(angle));
                initialMasses[index] = random.NextDouble(0.01, 0.1);

                double softenedRadiusSquared = radius * radius + burstManager.epsilon * burstManager.epsilon;
                double attractiveAcceleration = burstManager.alphaEff * centralMass * radius /
                    (softenedRadiusSquared * math.sqrt(softenedRadiusSquared));
                double circularSpeed = math.sqrt(math.max(0.0, attractiveAcceleration * radius));
                initialVelocities[index] = new double3(
                    -circularSpeed * math.sin(angle),
                    0.0,
                    circularSpeed * math.cos(angle));
            }

            burstManager.Initialize(initialPositions, initialVelocities, initialMasses);
            int batchCount = Mathf.CeilToInt(count / (float)MaxInstancesPerDraw);
            renderBatches = new Matrix4x4[batchCount][];
            for (int batch = 0; batch < batchCount; batch++)
            {
                int batchSize = Mathf.Min(MaxInstancesPerDraw, count - batch * MaxInstancesPerDraw);
                renderBatches[batch] = new Matrix4x4[batchSize];
            }
        }

        private void NativeArrayToRenderBatches(Unity.Collections.NativeArray<double3> positions)
        {
            Vector3 scale = Vector3.one * 0.4f;
            for (int batch = 0; batch < renderBatches.Length; batch++)
            {
                int offset = batch * MaxInstancesPerDraw;
                for (int index = 0; index < renderBatches[batch].Length; index++)
                {
                    double3 position = positions[offset + index];
                    renderBatches[batch][index].SetTRS(
                        new Vector3((float)position.x, (float)position.y, (float)position.z),
                        Quaternion.identity,
                        scale);
                }
            }
        }
    }
}
