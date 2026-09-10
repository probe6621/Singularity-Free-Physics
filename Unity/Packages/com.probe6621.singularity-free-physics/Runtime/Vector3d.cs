using System;
using UnityEngine;

namespace SingularityFreePhysics
{
    /// <summary>Small double-precision vector type used by the simulation state.</summary>
    [Serializable]
    public struct Vector3d
    {
        public double x;
        public double y;
        public double z;

        public Vector3d(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vector3d(Vector3 value)
        {
            x = value.x;
            y = value.y;
            z = value.z;
        }

        public static Vector3d zero => default;

        public static Vector3d operator +(Vector3d left, Vector3d right) =>
            new Vector3d(left.x + right.x, left.y + right.y, left.z + right.z);

        public static Vector3d operator -(Vector3d left, Vector3d right) =>
            new Vector3d(left.x - right.x, left.y - right.y, left.z - right.z);

        public static Vector3d operator -(Vector3d value) =>
            new Vector3d(-value.x, -value.y, -value.z);

        public static Vector3d operator *(Vector3d value, double scalar) =>
            new Vector3d(value.x * scalar, value.y * scalar, value.z * scalar);

        public static Vector3d operator *(double scalar, Vector3d value) => value * scalar;

        public static Vector3d operator /(Vector3d value, double scalar) =>
            new Vector3d(value.x / scalar, value.y / scalar, value.z / scalar);

        public double SqrMagnitude() => x * x + y * y + z * z;

        public static double Dot(Vector3d left, Vector3d right) =>
            left.x * right.x + left.y * right.y + left.z * right.z;

        public bool IsFinite() =>
            !double.IsNaN(x) && !double.IsInfinity(x) &&
            !double.IsNaN(y) && !double.IsInfinity(y) &&
            !double.IsNaN(z) && !double.IsInfinity(z);

        public Vector3 ToVector3() => new Vector3((float)x, (float)y, (float)z);
    }
}
