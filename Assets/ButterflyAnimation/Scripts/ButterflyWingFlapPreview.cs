using UnityEngine;

namespace Dexter.Butterfly
{
    /// <summary>
    /// Procedural wing flap used at runtime. Supports slowing or pausing during rests.
    /// </summary>
    public sealed class ButterflyWingFlapPreview : MonoBehaviour
    {
        [SerializeField] private float flapSpeed = 6f;
        [SerializeField] private float flapAngle = 28f;

        private Transform wingUpperLeft;
        private Transform wingUpperRight;
        private Transform wingLowerLeft;
        private Transform wingLowerRight;
        private Quaternion upperLeftRest;
        private Quaternion upperRightRest;
        private Quaternion lowerLeftRest;
        private Quaternion lowerRightRest;

        public float FlapSpeedMultiplier { get; set; } = 1f;
        public float FlapAngleMultiplier { get; set; } = 1f;

        public void StopFlapping()
        {
            FlapSpeedMultiplier = 0f;
            FlapAngleMultiplier = 0f;

            ApplyFlap(wingUpperLeft, upperLeftRest, 0f);
            ApplyFlap(wingUpperRight, upperRightRest, 0f);
            ApplyFlap(wingLowerLeft, lowerLeftRest, 0f);
            ApplyFlap(wingLowerRight, lowerRightRest, 0f);
        }

        private void Awake()
        {
            CacheWing("wing_upper.L", ref wingUpperLeft, ref upperLeftRest);
            CacheWing("wing_upper.R", ref wingUpperRight, ref upperRightRest);
            CacheWing("wing_lower.L", ref wingLowerLeft, ref lowerLeftRest);
            CacheWing("wing_lower.R", ref wingLowerRight, ref lowerRightRest);
        }

        private void CacheWing(string boneName, ref Transform bone, ref Quaternion restRotation)
        {
            bone = FindBone(boneName);
            if (bone != null)
                restRotation = bone.localRotation;
        }

        private Transform FindBone(string boneName)
        {
            foreach (Transform child in GetComponentsInChildren<Transform>(true))
            {
                if (child.name == boneName)
                    return child;
            }

            return null;
        }

        private void LateUpdate()
        {
            float speed = flapSpeed * FlapSpeedMultiplier;
            float angle = speed <= 0.01f
                ? 0f
                : Mathf.Sin(Time.time * speed) * flapAngle * FlapAngleMultiplier;

            ApplyFlap(wingUpperLeft, upperLeftRest, angle);
            ApplyFlap(wingUpperRight, upperRightRest, -angle);
            ApplyFlap(wingLowerLeft, lowerLeftRest, angle * 0.55f);
            ApplyFlap(wingLowerRight, lowerRightRest, -angle * 0.55f);
        }

        private static void ApplyFlap(Transform bone, Quaternion restRotation, float angle)
        {
            if (bone == null)
                return;

            bone.localRotation = restRotation * Quaternion.Euler(0f, 0f, angle);
        }
    }
}
